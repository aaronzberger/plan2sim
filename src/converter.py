#!/usr/bin/python3

from __future__ import annotations
import json
from typing import Any, Type

from termcolor import colored

from helpers import Action, Constraint, Plan, State
from interval_tree import IntervalTree


def field_type(str) -> Type:
    return {
        'float': type(0.),
        'str': type(''),
        'int': type(0)
    }[str]


def deserialize(data: dict[str, Any]) -> Plan:
    actions: list[Action] = []
    for action in data['actions']:
        preconditions = [State(**p) for p in action['preconditions']]
        effects = [State(**e) for e in action['effects']]
        actions.append(Action(**action, preconditions=preconditions, effects=effects))

    constraints = [Constraint(**c) for c in data['constraints']]

    return Plan(actions=actions, constraints=constraints)


class Converter:
    def __init__(self, plan_file: str):
        self.plan_file = plan_file

        self.features: list[list[float | str]] = []

        self.action_table: dict[str, Action] = {}
        self.constraint_table: dict[str, Constraint] = {}

    def convert(self):
        'Iterate through the planned file and register actions to the timeline'
        plan_dict: dict[str, Any] = json.load(open(self.plan_file))
        plan: Plan = deserialize(plan_dict)

        self.features = [[action.lst, action.lft, action.id] for action in plan.actions]

        self.timeline = IntervalTree(self.features, 0, max([i[1] for i in self.features]))

        # Hash all actions by ID
        for action in plan.actions:
            self.action_table[action.id] = action

        # Hash all constraints by source ID
        for constraint in plan.constraints:
            self.constraint_table[constraint.source] = constraint

        print(colored('Registered {} actions successfully.'.format(len(self.action_table)), color='green'))
        print(colored('Created interval tree with {} nodes successfully.'.format(len(self.features)), color='green'))
