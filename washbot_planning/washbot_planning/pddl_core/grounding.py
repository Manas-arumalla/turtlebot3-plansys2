"""Ground a lifted STRIPS task into propositional actions.

Static predicates (those never touched by any effect, e.g. ``connected``)
are evaluated once against the initial state and pruned from the ground
actions. This keeps the branching factor proportional to the world's
connectivity graph instead of the full location cross-product, which is what
makes the internal planner usable for the benchmark's larger instances.
"""

from __future__ import annotations

import itertools
from typing import Dict, FrozenSet, List, Set

from washbot_planning.pddl_core.model import (
    Atom,
    Domain,
    GroundAction,
    GroundTask,
    PddlError,
    Problem,
)


def _static_predicates(domain: Domain) -> Set[str]:
    dynamic = {
        literal.atom.predicate
        for action in domain.actions.values()
        for literal in action.effects
    }
    return set(domain.predicates) - dynamic


def _substitute(atom: Atom, binding: Dict[str, str]) -> Atom:
    return Atom(atom.predicate, tuple(binding.get(a, a) for a in atom.args))


def ground(domain: Domain, problem: Problem) -> GroundTask:
    """Instantiate every action schema over the problem's typed objects."""
    if problem.domain_name != domain.name:
        raise PddlError(
            f'problem "{problem.name}" targets domain "{problem.domain_name}", '
            f'but the parsed domain is "{domain.name}"')

    static_preds = _static_predicates(domain)
    static_atoms = frozenset(a for a in problem.init if a.predicate in static_preds)

    ground_actions: List[GroundAction] = []
    for schema in domain.actions.values():
        candidate_pools = [
            problem.objects_of_type(domain, param_type)
            for _, param_type in schema.parameters
        ]
        variables = [var for var, _ in schema.parameters]
        for combo in itertools.product(*candidate_pools):
            binding = dict(zip(variables, combo))

            preconditions: Set[Atom] = set()
            statically_impossible = False
            for literal in schema.preconditions:
                atom = _substitute(literal.atom, binding)
                if not literal.positive:
                    raise PddlError(
                        f'negative precondition {literal} in action '
                        f'"{schema.name}" is not supported')
                if atom.predicate in static_preds:
                    if atom not in static_atoms:
                        statically_impossible = True
                        break
                else:
                    preconditions.add(atom)
            if statically_impossible:
                continue

            add_effects: Set[Atom] = set()
            del_effects: Set[Atom] = set()
            for literal in schema.effects:
                atom = _substitute(literal.atom, binding)
                (add_effects if literal.positive else del_effects).add(atom)

            ground_actions.append(GroundAction(
                name=schema.name,
                args=tuple(combo),
                preconditions=frozenset(preconditions),
                add_effects=frozenset(add_effects),
                del_effects=frozenset(del_effects),
            ))

    dynamic_init: FrozenSet[Atom] = frozenset(
        a for a in problem.init if a.predicate not in static_preds)

    unreachable_goals = {a for a in problem.goal if a.predicate in static_preds}
    if unreachable_goals - static_atoms:
        missing = ', '.join(str(a) for a in sorted(unreachable_goals - static_atoms))
        raise PddlError(f'goal contains static atoms not present in :init: {missing}')

    return GroundTask(
        domain_name=domain.name,
        problem_name=problem.name,
        init=dynamic_init,
        goal=frozenset(a for a in problem.goal if a.predicate not in static_preds),
        actions=ground_actions,
        static_atoms=static_atoms,
    )
