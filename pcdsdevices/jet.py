"""
Module for the liquid jet classes.
"""
from ophyd import Component as Cpt

from .device import GroupDevice
from .device import UnrelatedComponent as UCpt
from .epics_motor import IMS, BeckhoffAxis_Pre140
from .interface import BaseInterface


class Injector(BaseInterface, GroupDevice):
    """
    Positioner for liquid jet Injector.

    Consists of 3 control motors, one for each of x, y, and z.

    Parameters
    ----------
    x_prefix : str
        Prefix for the coarse control motor in the X direction.

    y_prefix : str
        Prefix for the coarse control motor in the Y direction.

    z_prefix : str
        Prefix for the coarse control motor in the Z direction.
    """

    x = UCpt(IMS)
    y = UCpt(IMS)
    z = UCpt(IMS)

    def __init__(self, *, name, **kwargs):
        UCpt.collect_prefixes(self, kwargs)
        super().__init__('', name=name, **kwargs)


class InjectorWithFine(Injector):
    """
    Positioner for liquid jet Injector, with fine control.

    Consists of 6 control motors, two for each of x, y, and z.
    Each dimension has both a coarse and a fine motor.

    Parameters
    ----------
    x_prefix : str
        Prefix for the coarse control motor in the X direction.

    y_prefix : str
        Prefix for the coarse control motor in the Y direction.

    z_prefix : str
        Prefix for the coarse control motor in the Z direction.

    fine_x_prefix : str
        Prefix for the fine control motor in the X direction.

    fine_y_prefix : str
        Prefix for the fine control motor in the Y direction.

    fine_z_prefix : str
        Prefix for the fine control motor in the Z direction.
    """

    fine_x = UCpt(IMS)
    fine_y = UCpt(IMS)
    fine_z = UCpt(IMS)


class BeckhoffJetManipulator(BaseInterface, GroupDevice):
    """Jet Manipulator controlled by Beckhoff PLC."""

    tab_component_names = True

    x = Cpt(BeckhoffAxis_Pre140, ':X', kind='normal')
    y = Cpt(BeckhoffAxis_Pre140, ':Y', kind='normal')
    z = Cpt(BeckhoffAxis_Pre140, ':Z', kind='normal')


class BeckhoffJetSlits(BaseInterface, GroupDevice):
    """Pair of Beckhoff-controlled slits where each blade has X & Y motors."""
    tab_component_names = True

    top_x = Cpt(BeckhoffAxis_Pre140, ':TOP_X', kind='normal')
    top_y = Cpt(BeckhoffAxis_Pre140, ':TOP_Y', kind='normal')
    bot_x = Cpt(BeckhoffAxis_Pre140, ':BOT_X', kind='normal')
    bot_y = Cpt(BeckhoffAxis_Pre140, ':BOT_Y', kind='normal')


class BeckhoffJet(BaseInterface, GroupDevice):
    """
    Full liquid jet setup controlled by a Beckhoff PLC.

    This includes three axes for the jet manipulator, two axes for each of two
    slits, and a single axis for the detector. Each of their PVs will be
    inferred from the base prefix.

    Parameters
    ----------
    prefix : str, optional
        Liquid jet base PV.

    name : str
        A name to refer to the device.
    """

    tab_component_names = True

    jet = Cpt(BeckhoffJetManipulator, ':JET', kind='normal')
    ss = Cpt(BeckhoffJetSlits, ':SS', kind='normal')
    vh_epix_x = Cpt(BeckhoffAxis_Pre140, ':DET:X', kind='normal')
