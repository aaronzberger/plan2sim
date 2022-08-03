#!/usr/bin/python3

from __future__ import annotations

import json
from typing import Any

from termcolor import colored

from helpers import Action, Constraint, Plan, State
from interval_tree import IntervalTree


def deserialize(data: dict[str, Any]) -> Plan:
    actions: list[Action] = []
    for action in data['actions']:
        preconditions = [State(**p) for p in action['preconditions']]
        effects = [State(**e) for e in action['effects']]

        if len(effects) > 1:
            raise ValueError('Conversion failed: an action can only model one state change, ' +
                             'and can therefore only have one effect.')

        actions.append(Action(**action, preconditions=preconditions, effects=effects))

    constraints = [Constraint(**c) for c in data['constraints']]

    return Plan(actions=actions, constraints=constraints)


class Converter:
    def __init__(self, plan_file: str):
        self.plan_file = plan_file

        self._features: list[list[float | str]] = []

        self.action_table: dict[str, Action] = {}
        self.constraint_table: dict[str, list[Constraint]] = {}

    def convert(self):
        'Iterate through the planned file and register actions to the timeline'
        plan_dict: dict[str, Any] = json.load(open(self.plan_file))
        plan: Plan = deserialize(plan_dict)

        self._features = [[action.lst, action.lft, action.id] for action in plan.actions]
        self.timeline = IntervalTree(self._features, 0, max([i[1] for i in self._features]))

        # Hash all actions by ID
        self.action_table = {action.id: action for action in plan.actions}

        # Hash all constraints by source ID
        for constraint in plan.constraints:
            if constraint.type != 'BEFORE':
                raise ValueError('Conversion failed: only constraints of type \'BEFORE\' are currently supported')
            if constraint.source in self.constraint_table:
                self.constraint_table[constraint.source].append(constraint)
            else:
                self.constraint_table[constraint.source] = [constraint]

        print(colored('Registered {} actions successfully.'.format(len(self.action_table)), color='green'))
        print(colored('Created interval tree with {} nodes successfully.'.format(len(self._features)), color='green'))
