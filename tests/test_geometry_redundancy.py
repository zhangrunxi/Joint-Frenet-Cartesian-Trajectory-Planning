"""The redundancy expressions behind Fig. 7 of the manuscript.

The manuscript prints three redundancy values for the minibus: 0.77 m and
1.16 m for the three-circle covering, 1.08 m and 0.48 m for the five-circle
covering, and 0.43 m for the proposed capsule-plus-corner-circle model.

All three follow from the manuscript's own Eq. 11-14 and Eq. 22:

    r(N)  = (W/2) * sqrt(1 + lambda^2 / N^2),   lambda = L / W
    dL(N) = 2 * r(N) - L / N
    dW(N) = 2 * r(N) - W
    hybrid: dL = dW = (W/2) * (sqrt(2) - 1)

The three-circle covering is Eq. 11-14 at N = 3, the five-circle covering is
the same expression at N = 5, and Eq. 12 is the N = 3 reduction of it.
"""

import importlib.util
import pathlib
import sys

import numpy as np
import pytest

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "measure_geometry_redundancy.py"
_spec = importlib.util.spec_from_file_location("measure_geometry_redundancy", _SCRIPT)
_redundancy = importlib.util.module_from_spec(_spec)
sys.modules["measure_geometry_redundancy"] = _redundancy
_spec.loader.exec_module(_redundancy)

MINIBUS = (7.5, 2.1)


def test_three_circle_matches_the_printed_minibus_value():
    delta_l, delta_w = _redundancy.circle_redundancy(*MINIBUS, 3)
    assert delta_l == pytest.approx(0.77, abs=0.01)
    assert delta_w == pytest.approx(1.16, abs=0.01)


def test_five_circle_matches_the_printed_minibus_value():
    delta_l, delta_w = _redundancy.five_circle_redundancy(*MINIBUS)
    assert delta_l == pytest.approx(1.08, abs=0.01)
    assert delta_w == pytest.approx(0.48, abs=0.01)


def test_five_circle_is_the_five_circle_case_of_the_general_expression():
    """The five-circle covering is Eq. 11-14 evaluated at N = 5.

    This is the relation that makes the reconstruction match the printed
    value. A five-circle layout with a fixed circle radius does not.
    """
    length, width = MINIBUS
    assert _redundancy.five_circle_redundancy(length, width) == pytest.approx(
        _redundancy.circle_redundancy(length, width, 5)
    )
    # The fixed-radius layout that was tried first does not reproduce it.
    fixed_radius = 0.5 * width * np.sqrt(2.0)
    assert 2 * fixed_radius - length / 5 != pytest.approx(1.08, abs=0.01)


def test_proposed_model_matches_the_printed_minibus_value():
    delta_l, delta_w = _redundancy.hybrid_redundancy(*MINIBUS)
    assert delta_l == pytest.approx(0.43, abs=0.01)
    assert delta_w == pytest.approx(0.43, abs=0.01)


def test_equation_12_is_the_three_circle_reduction():
    """Eq. 12 is the N = 3 case of Eq. 11-14."""
    length, width = MINIBUS
    aspect = length / width
    delta_l, delta_w = _redundancy.circle_redundancy(length, width, 3)
    # (L/3) * (sqrt(1 + 9/lambda^2) - 1)
    expected_l = (length / 3.0) * (np.sqrt(1.0 + 9.0 / aspect**2) - 1.0)
    # (sqrt(1 + lambda^2/9) - 1) * W
    expected_w = (np.sqrt(1.0 + aspect**2 / 9.0) - 1.0) * width
    assert delta_l == pytest.approx(expected_l)
    assert delta_w == pytest.approx(expected_w)


def test_the_proposed_model_is_never_larger_in_the_longitudinal_direction():
    """Longitudinally the proposed model is tighter on every vehicle type."""
    for name, (length, width) in _redundancy.VEHICLES.items():
        hybrid_l, _ = _redundancy.hybrid_redundancy(length, width)
        for count in (3, 5):
            delta_l, _ = _redundancy.circle_redundancy(length, width, count)
            assert hybrid_l <= delta_l, name


def test_the_lateral_claim_has_exceptions_on_the_compact_vehicles():
    """The lateral claim has exceptions, and they are the compact vehicles.

    The proposed model is laterally tighter than the three-circle covering on
    every vehicle type except the Smart, and not than the five-circle covering
    for the Smart, Sedan, SUV and the test vehicle of Table 1: for those, the
    five circles are laterally tighter than the proposed model, because the
    circle layouts waste little lateral extent once the aspect ratio is small.
    The exceptions are stated in the manuscript rather than hidden, so this
    test pins them down.
    """
    exceptions = []
    for name, (length, width) in _redundancy.VEHICLES.items():
        hybrid_l, hybrid_w = _redundancy.hybrid_redundancy(length, width)
        for count in (3, 5):
            delta_l, delta_w = _redundancy.circle_redundancy(length, width, count)
            if hybrid_w > delta_w:
                exceptions.append((name, count))
    assert exceptions == [
        ("Smart", 3),
        ("Smart", 5),
        ("Sedan", 5),
        ("SUV", 5),
        ("Bus (Table 1)", 5),
    ]


def test_the_test_vehicle_of_table_1_is_longitudinally_tighter_than_both_coverings():
    """For the bus geometry of Table 1 the proposed model is longitudinally
    the tightest of the three, and laterally tighter than the three-circle
    covering but not than the five-circle one."""
    length, width = 6.50, 2.30
    hybrid_l, hybrid_w = _redundancy.hybrid_redundancy(length, width)
    three_l, three_w = _redundancy.circle_redundancy(length, width, 3)
    five_l, five_w = _redundancy.circle_redundancy(length, width, 5)
    assert hybrid_l < three_l < five_l
    assert hybrid_w < three_w
    assert hybrid_w > five_w
