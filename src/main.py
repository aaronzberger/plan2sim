#!/usr/bin/python3

import rospy
import sys
import os
from converter import Converter, Action

import actionlib
import plan2sim.msg as msg


def server(action: Action):
    if action.robot == 'UR5A':
        return 'ur5a_rail' if 'rail' in action.name else 'ur5a_arm'
    if action.robot == 'UR5B':
        return 'ur5b_rail' if 'rail' in action.name else 'ur5b_arm'
    if action.robot == 'FF1':
        return 'free_flyer' if 'traverse' in action.name else 'free_flyer_arm'


class Plan2Sim:
    def __init__(self, plan_file):
        self.converter_node = Converter(plan_file)
        self.converter_node.convert()
        self.interval_tree = self.converter_node.get_timeline()
        self.action_table = self.converter_node.get_action_table()

        self.constraints = []

        self.action_clients = {
            'ur5a_arm': actionlib.SimpleActionClient('ur5a_arm', msg.PerformTaskAction),
            'ur5a_rail': actionlib.SimpleActionClient('ur5a_rail', msg.PerformTaskAction),
            'ur5b_arm': actionlib.SimpleActionClient('ur5b_arm', msg.PerformTaskAction),
            'ur5b_rail': actionlib.SimpleActionClient('ur5b_rail', msg.PerformTaskAction),
            'free_flyer': actionlib.SimpleActionClient('free_flyer', msg.PerformTaskAction),
            'free_flyer_arm': actionlib.SimpleActionClient('free_flyer', msg.PerformTaskAction)
        }

        # for i, client in enumerate(self.action_clients):
        #     print('Waiting for {} server ({}/{})... '.format(
        #         client, i + 1, len(self.action_clients)), end='', flush=True)
        #     self.action_clients[client].wait_for_server()
        #     print('CONNECTED')
        rospy.loginfo('All servers connected.')

        # Begin the planner
        self.actions_completed = set()
        self.actions_executing = set()
        self.start_time = rospy.get_rostime()
        rospy.Timer(rospy.Duration(secs=1), self.planner)

    def planner(self, event: rospy.timer.TimerEvent):
        def run(action: Action):
            goal = msg.PerformTaskGoal(task=msg.action(
                name=action.name, start_time=action.start_time, end_time=action.end_time,
                start=action.start, end=action.end))

            # Send the goal to the server and register the feedback and results callbacks
            client = server(action)
            if client:
                self.action_clients[client].send_goal(
                    goal, feedback_cb=(lambda fb: print('{} action is {}% done'.format(action.name, fb.percent_complete))),
                    done_cb=(lambda _, res: print(
                        '{} completed at {} seconds at position {}'.format(
                            action.name, res.result.time_ended, res.result.final_position))))
            else:
                print('Unknown implementation of action {}'.format(action.name))
            # self.action_clients[action.name].wait_for_result(rospy.Duration.from_sec(5))

        if event.last_real is None:
            event.last_real = self.start_time

        # Get the actions that should be running now from the interval tree
        current_actions = self.interval_tree.find_range(
            [(event.last_real - self.start_time).to_sec(), (event.current_real - self.start_time).to_sec()])

        for action in current_actions:
            if action not in self.actions_completed | self.actions_executing:

                # Deal with all constraints
                run(self.action_table[action])


if __name__ == '__main__':
    rospy.init_node('plan2sim', log_level=rospy.INFO)

    if len(sys.argv) != 2 or not os.path.exists(sys.argv[1]):
        rospy.logfatal('Usage: rosrun plan2sim [PLAN FILE]')
        sys.exit()

    plan2sim_node = Plan2Sim(sys.argv[1])

    rospy.spin()
