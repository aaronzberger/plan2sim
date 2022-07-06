#!/usr/bin/python3

import os
import sys

import actionlib
import rospy
from std_srvs.srv import Trigger
from termcolor import colored

from converter import Action, Converter
import plan2sim.msg as msg
import custom_actions
import system_controllers.msg
from plan2sim.msg import PerformTaskResult


def server(action: Action):
    if action.robot == 'UR5A':
        return 'ur5a_rail' if 'rail' in action.name else 'ur5a_arm'
    if action.robot == 'UR5B':
        return 'ur5b_rail' if 'rail' in action.name else 'ur5b_arm'
    if action.robot == 'FF1':
        return 'free_flyer' if 'traverse' in action.name else 'free_flyer_arm'


class Plan2Sim:
    def __init__(self, plan_file: str):
        self.converter_node = Converter(plan_file)
        self.converter_node.convert()
        self.interval_tree = self.converter_node.get_timeline()
        self.action_table = self.converter_node.get_action_table()

        self.constraints = []

        self.action_clients = {
            'ur5a_arm': actionlib.SimpleActionClient('ur5a_arm', msg.PerformTaskAction),
            'ur5a_gripper': actionlib.SimpleActionClient('ur5a_gripper', msg.PerformTaskAction),
            'ur5a_rail': actionlib.SimpleActionClient('ur5a_rail', msg.PerformTaskAction),
            'ur5b_arm': actionlib.SimpleActionClient('ur5b_arm', msg.PerformTaskAction),
            'ur5b_gripper': actionlib.SimpleActionClient('ur5b_gripper', msg.PerformTaskAction),
            'ur5b_rail': actionlib.SimpleActionClient('ur5b_rail', msg.PerformTaskAction),
            'free_flyer': actionlib.SimpleActionClient('free_flyer', msg.PerformTaskAction),
            'free_flyer_arm': actionlib.SimpleActionClient('free_flyer', msg.PerformTaskAction)
        }

        for i, client in enumerate(self.action_clients):
            print(colored('Waiting for {} server ({}/{})... '.format(
                client, i + 1, len(self.action_clients)), color='yellow', attrs=['blink']), end='', flush=True)
            self.action_clients[client].wait_for_server()
            print('\r' + colored('Waiting for {} server ({}/{})... '.format(
                client, i + 1, len(self.action_clients)), color='yellow'), end='', flush=True)
            print(colored('CONNECTED', color='green'))
        print(colored('All servers connected.', color='green'))

        # Start the simulation
        rospy.wait_for_service('start_sim', timeout=rospy.Duration(secs=20))
        sim_response = rospy.ServiceProxy('start_sim', Trigger)()
        if not sim_response.success:
            raise ConnectionRefusedError('Could not start the simulation successfully.')

        # Move the subsystems to their default locations
        def done_cb(_, res: PerformTaskResult):
            if not res.result.succeeded:
                print(colored('Failed to complete move to {}'.format(res.result.name), color='red'))

        def set_to_initial(name, position):
            self.action_clients[name].send_goal(msg.PerformTaskGoal(task=msg.action(
                name=name + '_initial', start_time=0, end_time=10,
                start='', end=position)), done_cb=done_cb)

        set_to_initial('ur5a_arm', 'contracted')
        set_to_initial('ur5b_arm', 'contracted')
        set_to_initial('ur5b_rail', 'r-blk10')
        set_to_initial('ur5a_rail', 'r-blk1')

        rospy.sleep(rospy.Duration(10))

        # Begin the planner
        self.actions_completed = set()
        self.actions_executing = set()
        self.start_time = rospy.get_rostime()
        rospy.Timer(rospy.Duration(secs=1), self.planner)

        # The running offset needed to satisfy the action constraints
        self.time_offset = rospy.Duration(secs=0)

    def planner(self, event: rospy.timer.TimerEvent):
        def run(action: Action):
            '''Send a task to its controller and manage feedback and results'''

            def feedback_cb(fb):
                '''Handle any updates from the controller during this action's execution'''
                # print('{} action is {}% done'.format(action.name, fb.percent_complete))

            def action_complete(_, res: PerformTaskResult):
                '''Handle the final result from the controller after this action has finished'''
                # print('received action complete in plan2sim callback', res)
                if res.result.succeeded:
                    print(colored('{} completed at {:.2f} seconds'.format(
                        res.result.name, res.result.time_ended), color='green'), end='')
                    print(colored(' at position {}'.format(res.result.final_position), color='green')
                          if res.result.final_position != 'n/a' else '')
                else:
                    print(colored('{} failed to complete fully and ended at {:.2f}'.format(
                        res.result.name, res.result.time_ended), color='yellow'))
                self.actions_executing.remove(res.result.name)
                self.actions_completed.add(res.result.name)

            print(colored('starting action {}'.format(action.name), color='cyan'))

            goal = msg.PerformTaskGoal(task=msg.action(
                name=action.name, start_time=action.start_time + self.time_offset.to_sec(),
                end_time=action.end_time + self.time_offset.to_sec(),
                start=action.start, end=action.end))

            # Send the goal to the server and register the callbacks
            client = server(action)
            self.actions_executing.add(action.name)

            if client:
                self.action_clients[client].cancel_goal()
                self.action_clients[client].send_goal(
                    goal, feedback_cb=feedback_cb, done_cb=action_complete)
            else:
                custom_actions.execute(action.name)
                action_complete(None, PerformTaskResult(result=system_controllers.msg.result(
                    name=action.name, time_ended=action.start_time, final_position='n/a', succeeded=True)))

        if event.last_real is None:
            event.last_real = self.start_time

        # Get the actions that should be running now from the interval tree
        print('executing actions in time range [{:3.0f}:{:3.0f}]'.format(
            ((event.last_real - self.time_offset) - self.start_time).to_sec(),
            ((event.current_real - self.time_offset) - self.start_time).to_sec()))
        current_actions = self.interval_tree.find_range(
            [((event.last_real - self.time_offset) - self.start_time).to_sec(),
             ((event.current_real - self.time_offset) - self.start_time).to_sec()])
        for action in current_actions:
            if action not in self.actions_completed | self.actions_executing:
                # If all constrains are met, the intersection between the set of all completed actions and the
                # set of all constraints will be the same size as the set of all constraints
                constraints = set(
                        [i.action for i in self.action_table[action].constraints if i.constraint == 'after'])
                constraints_met = len(self.actions_completed & constraints) == len(constraints)
                if not constraints_met:
                    print(colored(
                        '{}\'s constraints failed. Delaying planner until met'.format(
                            action), color='red'), end='', flush=True)
                    counter = 0
                    while not constraints_met:
                        # Delay all future actions until the necessary constraint actions have finished
                        rospy.Rate(10).sleep()
                        self.time_offset += rospy.Duration(secs=0.1)
                        if counter % 10 == 0:
                            print(colored('.', color='red'), end='', flush=True)
                        counter += 1
                        constraints_met = len(self.actions_completed & constraints) == len(constraints)

                    print(colored('\rConstraint met...', color='green'), flush=True)

                run(self.action_table[action])


if __name__ == '__main__':
    rospy.init_node('plan2sim', log_level=rospy.INFO)

    if len(sys.argv) != 2 or not os.path.exists(sys.argv[1]):
        rospy.logfatal('Usage: rosrun plan2sim [PLAN FILE]')
        sys.exit()

    plan2sim_node = Plan2Sim(sys.argv[1])

    rospy.spin()
