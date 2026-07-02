"""Data model for the STRIPS + :typing fragment of PDDL."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple


@dataclass(frozen=True, order=True)
class Atom:
    """A (possibly ground) predicate application, e.g. ``(robot_at r1 basin)``."""

    predicate: str
    args: Tuple[str, ...] = ()

    def __str__(self) -> str:
        if self.args:
            return '({} {})'.format(self.predicate, ' '.join(self.args))
        return '({})'.format(self.predicate)


@dataclass(frozen=True)
class Literal:
    """An atom with a sign; negated literals only appear in preconditions/effects."""

    atom: Atom
    positive: bool = True

    def __str__(self) -> str:
        return str(self.atom) if self.positive else '(not {})'.format(self.atom)


@dataclass
class ActionSchema:
    """A lifted action: parameters are typed variables like ``?from - location``."""

    name: str
    parameters: List[Tuple[str, str]]  # (variable, type)
    preconditions: List[Literal]
    effects: List[Literal]

    @property
    def arity(self) -> int:
        return len(self.parameters)


@dataclass
class Domain:
    name: str
    requirements: List[str]
    # child type -> parent type ('object' is the implicit root)
    types: Dict[str, str]
    predicates: Dict[str, List[Tuple[str, str]]]
    actions: Dict[str, ActionSchema]

    def type_and_ancestors(self, type_name: str) -> List[str]:
        """Return ``type_name`` followed by its ancestors up to ``object``."""
        chain = [type_name]
        seen = {type_name}
        current = type_name
        while current in self.types and self.types[current] not in seen:
            current = self.types[current]
            chain.append(current)
            seen.add(current)
        if 'object' not in seen:
            chain.append('object')
        return chain


@dataclass
class Problem:
    name: str
    domain_name: str
    objects: Dict[str, str]  # object name -> type
    init: FrozenSet[Atom]
    goal: FrozenSet[Atom]

    def objects_of_type(self, domain: Domain, type_name: str) -> List[str]:
        """All objects whose type is ``type_name`` or a descendant of it."""
        result = [
            name for name, obj_type in self.objects.items()
            if type_name in domain.type_and_ancestors(obj_type)
        ]
        result.sort()
        return result


@dataclass(frozen=True)
class GroundAction:
    """A fully instantiated action, ready for search or validation."""

    name: str
    args: Tuple[str, ...]
    preconditions: FrozenSet[Atom]
    add_effects: FrozenSet[Atom]
    del_effects: FrozenSet[Atom]
    cost: float = 1.0

    @property
    def signature(self) -> str:
        return '({} {})'.format(self.name, ' '.join(self.args)) if self.args \
            else '({})'.format(self.name)

    def applicable(self, state: FrozenSet[Atom]) -> bool:
        return self.preconditions <= state

    def apply(self, state: FrozenSet[Atom]) -> FrozenSet[Atom]:
        return (state - self.del_effects) | self.add_effects


@dataclass
class GroundTask:
    """The result of grounding: everything search and validation need."""

    domain_name: str
    problem_name: str
    init: FrozenSet[Atom]
    goal: FrozenSet[Atom]
    actions: List[GroundAction]
    # Atoms of static predicates (true in init, never changed by any action).
    static_atoms: FrozenSet[Atom] = field(default_factory=frozenset)

    def goal_satisfied(self, state: FrozenSet[Atom]) -> bool:
        return self.goal <= state


class PddlError(Exception):
    """Raised for malformed PDDL input or unsupported constructs."""

    def __init__(self, message: str, context: Optional[str] = None):
        super().__init__(message if context is None else f'{message} (in {context})')
