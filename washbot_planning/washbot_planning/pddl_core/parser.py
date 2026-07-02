"""A small recursive-descent parser for the STRIPS + :typing fragment of PDDL.

The parser intentionally rejects constructs the internal planner cannot
handle (durative actions, numeric fluents, quantifiers, disjunctions) with a
clear error message instead of silently mis-planning. Temporal domains are
still checked for balanced structure via :func:`structural_check`.
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Union

from washbot_planning.pddl_core.model import (
    ActionSchema,
    Atom,
    Domain,
    Literal,
    PddlError,
    Problem,
)

SExpr = Union[str, List['SExpr']]

_UNSUPPORTED_IN_STRIPS = {
    ':durative-action', 'forall', 'exists', 'or', 'when', 'imply',
    'increase', 'decrease', 'assign', '>=', '<=', '>', '<', '=',
}


def tokenize(text: str) -> List[str]:
    """Split PDDL text into parenthesis and symbol tokens, dropping comments."""
    lines = []
    for line in text.splitlines():
        comment = line.find(';')
        lines.append(line if comment < 0 else line[:comment])
    cleaned = '\n'.join(lines)
    return cleaned.replace('(', ' ( ').replace(')', ' ) ').split()


def parse_sexpr(tokens: List[str]) -> SExpr:
    """Parse a single s-expression from a token list (consumed in place)."""
    if not tokens:
        raise PddlError('unexpected end of input')
    token = tokens.pop(0)
    if token == '(':
        expr: List[SExpr] = []
        while tokens and tokens[0] != ')':
            expr.append(parse_sexpr(tokens))
        if not tokens:
            raise PddlError('missing closing parenthesis')
        tokens.pop(0)  # ')'
        return expr
    if token == ')':
        raise PddlError('unexpected closing parenthesis')
    return token.lower()


def _parse_typed_list(items: List[str]) -> List[Tuple[str, str]]:
    """Parse ``a b - t1 c - t2`` into ``[(a, t1), (b, t1), (c, t2)]``."""
    result: List[Tuple[str, str]] = []
    pending: List[str] = []
    i = 0
    while i < len(items):
        if items[i] == '-':
            if i + 1 >= len(items):
                raise PddlError('dangling "-" in typed list')
            for name in pending:
                result.append((name, items[i + 1]))
            pending = []
            i += 2
        else:
            pending.append(items[i])
            i += 1
    for name in pending:
        result.append((name, 'object'))
    return result


def _parse_atom(expr: SExpr, context: str) -> Atom:
    if not isinstance(expr, list) or not expr or not all(isinstance(t, str) for t in expr):
        raise PddlError(f'expected an atom, got {expr!r}', context)
    return Atom(predicate=expr[0], args=tuple(expr[1:]))


def _parse_condition(expr: SExpr, context: str) -> List[Literal]:
    """Parse a conjunction of (possibly negated) atoms."""
    if not isinstance(expr, list) or not expr:
        raise PddlError(f'expected a condition, got {expr!r}', context)
    head = expr[0]
    if head in _UNSUPPORTED_IN_STRIPS:
        raise PddlError(f'unsupported construct "{head}" in STRIPS fragment', context)
    if head == 'and':
        literals: List[Literal] = []
        for sub in expr[1:]:
            literals.extend(_parse_condition(sub, context))
        return literals
    if head == 'not':
        if len(expr) != 2:
            raise PddlError('"not" takes exactly one argument', context)
        return [Literal(_parse_atom(expr[1], context), positive=False)]
    return [Literal(_parse_atom(expr, context), positive=True)]


def _parse_action(expr: List[SExpr]) -> ActionSchema:
    name = expr[1]
    if not isinstance(name, str):
        raise PddlError('action name must be a symbol')
    sections: Dict[str, SExpr] = {}
    i = 2
    while i < len(expr):
        key = expr[i]
        if not isinstance(key, str) or not key.startswith(':'):
            raise PddlError(f'expected a :keyword in action "{name}", got {key!r}')
        if i + 1 >= len(expr):
            raise PddlError(f'missing value for {key} in action "{name}"')
        sections[key] = expr[i + 1]
        i += 2

    params_expr = sections.get(':parameters', [])
    if not isinstance(params_expr, list) or not all(isinstance(t, str) for t in params_expr):
        raise PddlError(f'malformed :parameters in action "{name}"')
    parameters = _parse_typed_list(list(params_expr))

    preconditions = _parse_condition(
        sections.get(':precondition', ['and']), f'action "{name}" precondition')
    effects = _parse_condition(
        sections.get(':effect', ['and']), f'action "{name}" effect')
    return ActionSchema(name=name, parameters=parameters,
                        preconditions=preconditions, effects=effects)


def parse_domain(text: str) -> Domain:
    """Parse a STRIPS + :typing PDDL domain from text."""
    expr = parse_sexpr(tokenize(text))
    if not isinstance(expr, list) or not expr or expr[0] != 'define':
        raise PddlError('domain file must start with (define (domain ...))')

    name = ''
    requirements: List[str] = []
    types: Dict[str, str] = {}
    predicates: Dict[str, List[Tuple[str, str]]] = {}
    actions: Dict[str, ActionSchema] = {}

    for section in expr[1:]:
        if not isinstance(section, list) or not section:
            raise PddlError(f'malformed domain section: {section!r}')
        head = section[0]
        if head == 'domain':
            name = str(section[1])
        elif head == ':requirements':
            requirements = [str(r) for r in section[1:]]
        elif head == ':types':
            for child, parent in _parse_typed_list([str(t) for t in section[1:]]):
                types[child] = parent
        elif head == ':predicates':
            for pred in section[1:]:
                if not isinstance(pred, list) or not pred:
                    raise PddlError(f'malformed predicate: {pred!r}')
                pred_name = str(pred[0])
                predicates[pred_name] = _parse_typed_list([str(t) for t in pred[1:]])
        elif head == ':action':
            action = _parse_action(section)
            actions[action.name] = action
        elif head == ':durative-action':
            raise PddlError(
                'durative actions are not supported by the internal planner; '
                'use the POPF or PlanSys2 backend for temporal domains')
        elif head in (':functions', ':constants'):
            raise PddlError(
                f'"{head}" is not supported by the internal planner; '
                'use the POPF or PlanSys2 backend for temporal/numeric domains')
        else:
            raise PddlError(f'unknown domain section "{head}"')

    if not name:
        raise PddlError('domain has no name')
    return Domain(name=name, requirements=requirements, types=types,
                  predicates=predicates, actions=actions)


def parse_problem(text: str) -> Problem:
    """Parse a PDDL problem (ground init and conjunctive positive goal)."""
    expr = parse_sexpr(tokenize(text))
    if not isinstance(expr, list) or not expr or expr[0] != 'define':
        raise PddlError('problem file must start with (define (problem ...))')

    name = ''
    domain_name = ''
    objects: Dict[str, str] = {}
    init: List[Atom] = []
    goal: List[Atom] = []

    for section in expr[1:]:
        if not isinstance(section, list) or not section:
            raise PddlError(f'malformed problem section: {section!r}')
        head = section[0]
        if head == 'problem':
            name = str(section[1])
        elif head == ':domain':
            domain_name = str(section[1])
        elif head == ':objects':
            for obj, obj_type in _parse_typed_list([str(t) for t in section[1:]]):
                objects[obj] = obj_type
        elif head == ':init':
            for fact in section[1:]:
                if isinstance(fact, list) and fact and fact[0] == '=':
                    raise PddlError('numeric fluents in :init are not supported')
                init.append(_parse_atom(fact, ':init'))
        elif head == ':goal':
            if len(section) != 2:
                raise PddlError(':goal takes exactly one condition')
            for literal in _parse_condition(section[1], ':goal'):
                if not literal.positive:
                    raise PddlError('negative goals are not supported')
                goal.append(literal.atom)
        elif head == ':metric':
            continue  # metrics are meaningful to external planners only
        else:
            raise PddlError(f'unknown problem section "{head}"')

    if not name or not domain_name:
        raise PddlError('problem must declare a name and a :domain')
    return Problem(name=name, domain_name=domain_name, objects=objects,
                   init=frozenset(init), goal=frozenset(goal))


def structural_check(text: str) -> None:
    """Light syntax check (balanced s-expressions) for any PDDL file.

    Used for temporal domains that the full parser rejects on purpose:
    it catches unbalanced parentheses and empty files before the text is
    handed to an external planner.
    """
    tokens = tokenize(text)
    if not tokens:
        raise PddlError('file is empty')
    expr = parse_sexpr(tokens)
    if tokens:
        raise PddlError('trailing tokens after top-level expression')
    if not isinstance(expr, list) or not expr or expr[0] != 'define':
        raise PddlError('top-level expression must be (define ...)')
