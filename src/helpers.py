from __future__ import annotations
from dataclasses import dataclass

from typing import Literal


@dataclass
class State():
    predicate: str
    arguments: list[str]
    value: str


@dataclass
class Action():
    action: str
    id: str
    resources: list[str]
    preconditions: list[State]
    effects: list[State]
    est: int  # Earliest start
    lst: int  # Latest start
    eft: int  # Earliest finish
    lft: int  # Latest finish


@dataclass
class Constraint():
    type: Literal['BEFORE', 'AFTER']
    source: str
    sink: str
    lb: int  # Time lower bound
    ub: str  # Time upper bound


@dataclass
class Plan():
    actions: list[Action]
    constraints: list[Constraint]
