import logging

import numpy as np
import pytest
from ophyd.sim import FakeEpicsSignal, make_fake_device

from ..ladm import LADM
from ..positioner import FuncPositioner

logger = logging.getLogger(__name__)

alpha_r = 63 * np.pi/180
r = 2960.0  # first rail distance from sample to rail rod at 27deg
R = 6735.0
alpha_cos = np.cos(27 * np.pi/180)  # -> radians

motor_starting_position = 10


def motor_setup(motor):
    def sim_move(value, **kwargs):
        motor.motor_done_move.sim_put(0)
        motor.user_readback.sim_put(value)
        motor.motor_done_move.sim_put(1)

    motor.user_readback.sim_put(motor_starting_position)
    motor.user_readback.alarm_severity = 0
    motor.high_limit_travel.put(1000)
    motor.low_limit_travel.put(-1000)
    motor.user_setpoint.sim_set_limits((-1000, 1000))
    motor.motor_spg.sim_put(2)
    motor.user_setpoint.subscribe(sim_move)


@pytest.fixture(scope='function')
def fakeLADM():
    return sim_ladm()


def sim_ladm():
    fake_ladm = make_fake_device(LADM)
    test_ladm = fake_ladm('TEST:LAM', name='Fake LADM')
    motor_setup(test_ladm.x1)
    motor_setup(test_ladm.x2)
    motor_setup(test_ladm.y1)
    motor_setup(test_ladm.y2)
    motor_setup(test_ladm.z)
    motor_setup(test_ladm.bs6_r)
    motor_setup(test_ladm.bs6_t)
    motor_setup(test_ladm.bs2_r)
    motor_setup(test_ladm.bs2_t)
    motor_setup(test_ladm.bs10_r)
    motor_setup(test_ladm.bs10_t)
    test_ladm.theta_pv = FakeEpicsSignal('TEST:LAM:Theta',
                                         name='test_ladm_theta_pv')
    test_ladm.theta_pv.sim_put(10)
    test_ladm.gamma_pv = FakeEpicsSignal('TEST:LAM:Gamma',
                                         name='test _ladm_gamma_pv')
    test_ladm.gamma_pv.sim_put(10)
    test_ladm.theta = FuncPositioner(name="test_ladmTheta",
                                     move=test_ladm.moveTheta,
                                     get_pos=test_ladm.wmTheta,
                                     set_pos=test_ladm.setTheta)
    test_ladm.XT = FuncPositioner(name="test_ladmXT",
                                  move=test_ladm.moveX,
                                  get_pos=test_ladm.wmX,
                                  set_pos=test_ladm._setX)
    return test_ladm


def test_moveTheta(fakeLADM):
    ladm = fakeLADM
    ladm.moveTheta(10)


def test_wmTheta(fakeLADM):
    ladm = fakeLADM
    theta1, theta2, samz_offset, ca_samz_offset = ladm.wmTheta()
    test_theta1 = np.arctan(ladm.x1_wm() * (np.sin(alpha_r))**2 /
                                           (r-samz_offset * alpha_cos -
                                            ladm.x1.wm() * np.sin(2 * alpha_r)
                                            / 2))
    test_theta1 = test_theta1 * 180/np.pi
    test_theta1


def test_wmGamma(fakeLADM):
    ladm = fakeLADM
    gamma, message = ladm.wmGamma()
    assert ladm.y1.wm() == 10
    assert ladm.y2.wm() == 10
    assert gamma == 0


def test_mvrGamma(fakeLADM):
    ladm = fakeLADM
    ladm.mvrGamma(3, wait=True)  # theta=10, y1=ladm.y1, y2=ladm.y2)
    assert np.isclose(ladm.y1.wm(), (155.1270267 + 10))
    assert np.isclose(ladm.y2.wm(), (352.9663935 + 10))


def test_setTheta(fakeLADM):
    ladm = fakeLADM
    theta = 3 * np.pi/180
    x1_var = (r) * np.sin(theta)/(np.sin(alpha_r) * np.sin(alpha_r+theta))
    x2_var = (R) * np.sin(theta)/(np.sin(alpha_r) * np.sin(alpha_r+theta))
    dz_var = (r)/np.sin(alpha_r) - x1_var * np.sin(alpha_r)/np.sin(theta)
    ladm.setTheta(3)
    assert ladm.x1.wm() == x1_var
    assert ladm.x2.wm() == x2_var
    assert ladm.z.wm() == dz_var


def test_moveX(fakeLADM):
    ladm = fakeLADM
    ladm.moveX(3)
    # assert ladm.z ==
    # assert ladm.x1 ==
    # assert ladm.x2 ==


def test_tweakH(fakeLADM):
    ladm = fakeLADM
    ladm.tweakH(3)  # , z=ladm.z, x1=ladm.x1, x2=ladm.x2)


def test_mvrV(fakeLADM):
    ladm = fakeLADM
    ladm.mvrV(3)  # , y1=ladm.y1, y2=ladm.y2)
    assert ladm.y1.wm() == 13
    assert ladm.y2.wm() == 13


def test_ca_theta(fakeLADM):
    ladm = fakeLADM
    cal_theta, cal_x1, cal_x2, cal_dz, samz_offset = ladm.ca_theta(10)
    # cal_thetarad = cal_theta * np.pi/180

    assert cal_theta == 10
#   assert cal_x1 == ((r-samz_offset * alpha_cos) * np.sin(cal_thetarad)/
#   (np.sin(alpha_r) * np.sin(alpha_r + cal_thetarad)))
#   assert cal_x2 == ((R - samz_offset * alpha_cos) * np.sin(cal_thetarad)/
#   (np.sin(alpha_r) * np.sin(alpha_r + cal_thetarad)))
#   assert cal_dz == ((r - samz_offset * alpha_cos)/np.sin(alpha_r) - cal_x1 *
#   np.sin(alpha_r)/np.sin(cal_thetarad))
    assert samz_offset == 0


def test_stop(fakeLADM):
    ladm = fakeLADM
    ladm.x1.stop()
    assert ladm.x1.motor_spg.get() == 0
    ladm.x2.stop()
    assert ladm.x1.motor_spg.get() == 0
    ladm.y1.stop()
    assert ladm.x1.motor_spg.get() == 0
    ladm.y2.stop()
    assert ladm.x1.motor_spg.get() == 0
    ladm.z.stop()
    assert ladm.x1.motor_spg.get() == 0
