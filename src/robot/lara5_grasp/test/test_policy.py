import math
import pytest
from lara5_grasp.policy import contact_digits, freeze_contacts, next_positions, reached


def test_force_threshold_and_hand_digit_mapping():
    assert contact_digits(['right_thumb_distal_link','right_index_distal_link','table'], [.4,.05,2.],.15)=={'thumb'}


@pytest.mark.parametrize('links,forces', [(['a'],[]), (['a','a'],[1.,1.]), (['a'],[math.nan]), (['a'],[-1.])])
def test_bad_force_vectors_rejected(links,forces):
    with pytest.raises(ValueError): contact_digits(links,forces,.1)


def test_contact_preload_is_bounded_and_does_not_ratchet():
    frozen={}; target=[1.]*5+[.5]
    freeze_contacts(frozen, {'thumb'}, [0.]*6,target,.02)
    freeze_contacts(frozen, {'thumb'}, [.1]*6,target,.02)
    assert frozen=={0:.02,5:.02}
    assert next_positions([0.]*6,target,frozen,.05)==[.02,.05,.05,.05,.05,.02]


def test_noncontact_digits_advance_only_one_step():
    assert next_positions([.1]*6,[.2]*6,{},.03)==pytest.approx([.13]*6)
    assert reached([.18]*6,[.2]*6,{},.03)
    assert not reached([.1]*6,[.2]*6,{},.03)


def test_solver_limit_tolerance_never_becomes_an_out_of_bounds_command():
    assert next_positions([-.001]*6,[-.001]*6,{},.05)==[0.]*6
    with pytest.raises(ValueError):next_positions([-.1]*6,[0.]*6,{},.05)
