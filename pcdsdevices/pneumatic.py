"""
Pneumatic Classes.

This Module contains all the classes relating to Pneumatic Actuators
"""

from lightpath import LightpathState
from ophyd import Component as Cpt
from ophyd.status import Status

from pcdsdevices.interface import BaseInterface, LightpathMixin

from .signal import PytmcSignal


class BeckhoffPneumatic(BaseInterface, LightpathMixin):
    """
    Class containing basic Beckhoff Pneumatic support
    """
    lightpath_cpts = ['limit_switch_in', 'limit_switch_out']

    # readouts
    limit_switch_in = Cpt(PytmcSignal, ':PLC:bInLimitSwitch')
    limit_switch_out = Cpt(PytmcSignal, ':PLC:bOutLimitSwitch')

    retract_status = Cpt(PytmcSignal, ':bRetractDigitalOutput')
    insert_status = Cpt(PytmcSignal, ':bInsertDigitalOutput')

    # logic and supervisory
    interlock_ok = Cpt(PytmcSignal, 'bInterlockOK')
    insert_ok = Cpt(PytmcSignal, 'bInsertEnable')
    retract_ok = Cpt(PytmcSignal, 'bretractEnable')

    # commands
    insert_signal = Cpt(PytmcSignal, 'CMD:IN')
    retract_signal = Cpt(PytmcSignal, 'CMD:OUT')

    # returns
    busy = Cpt(PytmcSignal, ':bBusy')
    done = Cpt(PytmcSignal, ':bDone')
    reset = Cpt(PytmcSignal, ':bReset')
    error = Cpt(PytmcSignal, ':PLC:bError')
    error_id = Cpt(PytmcSignal, ':PLC:nErrorId')
    error_message = Cpt(PytmcSignal, ':PLC:sErrorMessage')
    position_state = Cpt(PytmcSignal, ':nPositionState', kind='hinted')

    def callback(self, *, old_value, value, **kwargs):
        if value:
            self.done.clear_sub(self.callback)
            if self.error.get():
                error = Exception(self.error_message.get())
                self.status.set_exception(error)
            else:
                self.status.set_finished()

    def insert(self, wait: bool = False, timeout: float = 10.0) -> Status:
        """
        Method for inserting Beckhoff Pneumatic Actuator
        """
        self.status = Status(timeout)

        if self.insert_ok.get():
            self.done.subscribe(self.callback)
            self.insert_signal.put(1)
        else:
            error = Exception("Insertion not permitted by PLC")
            self.status.set_exception(error)

        if wait:
            self.status.wait()
        return self.status

    def remove(self, wait: bool = False, timeout: float = 10.0) -> Status:
        """
        Method for removing Beckhoff Pneumatic Actuator
        """
        self.status = Status(timeout)

        if self.retract_ok.get():
            self.done.subscribe(self.callback)
            self.retract_signal.put(1)
        else:
            error = Exception("Removal not permitted by PLC")
            self.status.set_exception(error)

        if wait:
            self.status.wait()
        return self.status

    def calc_lightpath_state(self, limit_switch_in=None, limit_switch_out=None):
        trans = 0.0 if limit_switch_in and not limit_switch_out else 1.0

        status = LightpathState(
            inserted=bool(limit_switch_in),
            removed=bool(limit_switch_out),
            output={self.output_branches[0]: trans}
        )
        return status
