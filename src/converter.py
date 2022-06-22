#!/usr/bin/python3

from __future__ import annotations
from typing import Literal
from dataclasses import dataclass
import rospy
from interval_tree import IntervalTree


@dataclass
class Constraint():
    'Represent a constraint on an action, relative to another action'
    constraint: Literal["before", "after"]
    action: str


@dataclass
class Action():
    'Represents one action, taken by a single robot, at a specified time'
    name: str
    robot: str
    start_time: float
    end_time: float
    start: str
    end: str
    constraints: list[Constraint] = None

    def __init__(self, *args):
        it = iter(args)
        for name in it:
            setattr(self, name, next(it))
        for not_set in [i for i in list(self.__dataclass_fields__.keys()) if i not in list(self.__dict__.keys())]:
            setattr(self, not_set, None)


class Converter:
    def __init__(self, plan_file: str):
        self.plan_file = plan_file
        self.actions = []
        self.interval_tree = None
        self.mappings = {
            'robot': 'robot',
            'start-location': 'start',
            'end-location': 'end',
            'start-arm-pose': 'start',
            'end-arm-pose': 'end',
            'start-time': 'start_time',
            'end-time': 'end_time',
            'constraints': 'constraints',
            'arm-pose': '',
            'location': '',
            'gripper-status': ''
        }
        # print(type(Action().__dataclass_fields__['robot'].type))
        self.features = []

    def register_action(self, action_text: str):
        # First, add the action to the list of running actions
        args = []
        for item in action_text[1:]:
            name = self.mappings[item[:item.find(':')]]
            value = item[item.find(':') + 2:].replace('\'', '')
            if name != 'constraints' and name != '':
                value = (type(Action().__dataclass_fields__[name].type))(value)
            else:
                value = []

                # Find the before constraints
                constraints_before = item.find('before')
                while constraints_before != -1:
                    value.append(Constraint(
                        constraint='before',
                        action=item[constraints_before + 14:item.find(')', constraints_before + 14)]))
                    constraints_before = item.find('before', constraints_before + 1)

                # Find the after constraints
                constraints_before = item.find('after')
                while constraints_before != -1:
                    value.append(Constraint(
                        constraint='after',
                        action=item[constraints_before + 14:item.find(')', constraints_before + 13)]))
                    constraints_before = item.find('after', constraints_before + 1)
            args.extend([name, value])
        self.actions.append(Action('name', action_text[0][1:], *args))

        # Next, add the action to the timeline
        self.features.append([
            float(*(i[i.find(':') + 2:] for i in action_text if 'start-time' in i)),
            float(*(i[i.find(':') + 2:] for i in action_text if 'end-time' in i)),
            action_text[0][1:]])

    def convert(self):
        'Iterate through the planned file and register actions to the timeline'
        def combine_constraints(action_text: str):
            for item in action_text:
                if 'constraint' in item:
                    action_text[action_text.index(item):] = [''.join(action_text[action_text.index(item):])]
                    return action_text

        def clean_line(text: str):
            idx = text.find(';')
            return text.strip() if idx == -1 else text[:idx].strip()

        file = open(self.plan_file, "r")
        action_text = []
        paren_stack = 0
        for line in (clean_line(line) for line in file.readlines() if len(clean_line(line)) != 0):
            if paren_stack == 0:
                if line[0] == '(':
                    paren_stack += 1
                    action_text.append(line)
            else:
                paren_stack += line.count('(') - line.count(')')
                action_text.append(line)
            if paren_stack == 0 and len(action_text) != 0:
                self.register_action(combine_constraints(action_text=action_text))
                action_text = []
        self.timeline = IntervalTree(self.features, 0, max([i[1] for i in self.features]))

        rospy.loginfo('Registered %d actions successfully.', len(self.actions))
        rospy.loginfo('Created interval tree with %d nodes successfully.', len(self.features))

    def get_timeline(self):
        return self.timeline
