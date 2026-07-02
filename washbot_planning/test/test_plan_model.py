from dataclasses import dataclass

from washbot_planning.plan import Plan

POPF_OUTPUT = """
; Time 0.00
; States evaluated: 12
0.000: (move washbot dock hall)  [8.000]
8.001: (move washbot hall basin)  [6.000]
14.002: (clean washbot basin)  [15.000]
"""


def test_popf_output_parsed():
    plan = Plan.from_popf_output(POPF_OUTPUT)
    assert len(plan) == 3
    assert plan.steps[0].name == 'move'
    assert plan.steps[0].args == ('washbot', 'dock', 'hall')
    assert plan.steps[2].signature == '(clean washbot basin)'
    assert plan.makespan == 14.002 + 15.0


def test_popf_output_sorted_by_start_time():
    scrambled = """
    5.000: (b x)  [1.000]
    0.000: (a x)  [1.000]
    """
    plan = Plan.from_popf_output(scrambled)
    assert [s.name for s in plan.steps] == ['a', 'b']
    assert [s.index for s in plan.steps] == [0, 1]


def test_plansys2_items_parsed():
    @dataclass
    class FakeItem:
        time: float
        action: str
        duration: float

    items = [
        FakeItem(0.0, '(move washbot dock hall)', 8.0),
        FakeItem(8.0, '(clean washbot basin)', 15.0),
    ]
    plan = Plan.from_plansys2_items(items)
    assert len(plan) == 2
    assert plan.steps[1].args == ('washbot', 'basin')
    assert plan.source == 'plansys2'


def test_to_text_roundtrip():
    plan = Plan.from_popf_output(POPF_OUTPUT)
    again = Plan.from_popf_output(plan.to_text())
    assert [s.signature for s in again.steps] == [s.signature for s in plan.steps]
