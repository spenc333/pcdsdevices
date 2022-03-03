from __future__ import annotations

import enum
import operator
import os
import select
import shutil
import sys
import threading
import time
from functools import reduce
from typing import Iterator, Union

import ophyd
import pint
import prettytable

try:
    import termios
    import tty
except ImportError:
    tty = None
    termios = None


arrow_up = '\x1b[A'
arrow_down = '\x1b[B'
arrow_right = '\x1b[C'
arrow_left = '\x1b[D'
shift_arrow_up = '\x1b[1;2A'
shift_arrow_down = '\x1b[1;2B'
shift_arrow_right = '\x1b[1;2C'
shift_arrow_left = '\x1b[1;2D'
alt_arrow_up = '\x1b[1;3A'
alt_arrow_down = '\x1b[1;3B'
alt_arrow_right = '\x1b[1;3C'
alt_arrow_left = '\x1b[1;3D'
ctrl_arrow_up = '\x1b[1;5A'
ctrl_arrow_down = '\x1b[1;5B'
ctrl_arrow_right = '\x1b[1;5C'
ctrl_arrow_left = '\x1b[1;5D'
plus = '+'
minus = '-'


def is_input():
    """
    Utility to check if there is input available.

    Returns
    -------
    is_input : bool
        `True` if there is data in `sys.stdin`.
    """

    return select.select([sys.stdin], [], [], 1) == ([sys.stdin], [], [])


def get_input():
    """
    Waits for a single character input and returns it.

    You can compare the input to the keys stored in this module e.g.
    ``utils.arrow_up == get_input()``.

    Returns
    -------
    input : str
    """
    if termios is None:
        raise RuntimeError('Not supported on this platform')

    # Save old terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    # Stash a None here in case we get interrupted
    inp = None
    try:
        # Swap to cbreak mode to get raw inputs
        tty.setcbreak(sys.stdin.fileno())
        # Poll for input. This is interruptable with ctrl+c
        while (not is_input()):
            time.sleep(0.01)
        # Read the first character
        inp = sys.stdin.read(1)
        # Read more if we have a control sequence
        if inp == '\x1b':
            extra_inp = sys.stdin.read(2)
            inp += extra_inp
            # Read even more if we had a shift/alt/ctrl modifier
            if extra_inp == '[1':
                inp += sys.stdin.read(3)
    finally:
        # Restore the terminal to normal input mode
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return inp


ureg = None


def convert_unit(value, unit, new_unit):
    """
    One-line unit conversion.

    Parameters
    ----------
    value : float
        The starting value for the conversion.

    unit : str
        The starting unit for the conversion.

    new_unit : str
        The desired unit for the conversion.

    Returns
    -------
    new_value : float
        The starting value, but converted to the new unit.
    """

    global ureg
    if ureg is None:
        ureg = pint.UnitRegistry()

    expr = ureg.parse_expression(unit)
    return (value * expr).to(new_unit).magnitude


def ipm_screen(dettype, prefix, prefix_ioc):
    """
    Function to call the (pyQT) screen for an IPM box.

    Parameters
    ----------
    dettype : {'IPIMB', 'Wave8'}
        The type of detector being accessed.

    prefix : str
        The PV prefix associated with the device being accessed.

    prefix_ioc : str
        The PV prefix associated with the IOC running the device.
    """

    if (dettype == 'IPIMB'):
        executable = '/reg/g/pcds/controls/pycaqt/ipimb/ipimb'
    elif (dettype == 'Wave8'):
        executable = '/reg/g/pcds/pyps/apps/wave8/latest/wave8'
    else:
        raise ValueError('Unknown detector type')
    if shutil.which(executable) is None:
        raise EnvironmentError('%s is not on path, we cannot start the screen'
                               % executable)
    os.system('%s --base %s --ioc %s --evr %s &' %
              (executable, prefix, prefix_ioc, prefix+':TRIG'))


def get_component(obj):
    """
    Get the component that made the given object.

    Parameters
    ----------
    obj : ophyd.OphydItem
        The ophyd item for which to get the component.

    Returns
    -------
    component : ophyd.Component
        The component, if available.
    """
    if obj.parent is None:
        return None

    return getattr(type(obj.parent), obj.attr_name, None)


def schedule_task(func, args=None, kwargs=None, delay=None):
    """
    Use ophyd's dispatcher to schedule a task for later.

    This is basically the function I was hoping to find in ophyd.
    Schedules a task for the utility thread if we're in some arbitrary thread,
    schedules a task for the same thread if we're in one of ophyd's callback
    queues already.
    """
    if args is None:
        args = ()
    if kwargs is None:
        kwargs = {}

    dispatcher = ophyd.cl.get_dispatcher()

    # Check if we're already in an ophyd dispatcher thread
    current_thread = threading.currentThread()
    matched_thread = None
    for name, thread in dispatcher.threads.items():
        if thread == current_thread:
            matched_thread = name
            context = dispatcher.get_thread_context(matched_thread)
            break

    def schedule():
        if matched_thread is not None and context.event_thread is not None:
            # Put into same queue
            context.event_thread.queue.put((func, args, kwargs))
        else:
            # Put into utility queue
            dispatcher.schedule_utility_task(func, *args, **kwargs)

    if delay is None:
        # Do it right away
        schedule()
    else:
        # Do it later
        timer = threading.Timer(delay, schedule)
        timer.start()


def get_status_value(status_info, *keys, default_value='N/A'):
    """
    Get the value of a dictionary key.

    Parameters
    ----------
    status_info : dict
        Dictionary to look through.
    keys : list
        List of keys to look through with nested dictionarie.
    default_value : str
        A default value to return if the item value was not found.

    Returns
    -------
    value : dictionary item value
        Value of the last key in the `keys` list.
    """
    try:
        value = reduce(operator.getitem, keys, status_info)
        return value
    except KeyError:
        return default_value


def get_status_float(status_info, *keys, default_value='N/A', precision=4,
                     format='f', scale=1.0, include_plus_sign=False):
    """
    Get the value of a dictionary key.

    Format the value with the precision requested if it is a float value.

    Parameters
    ----------
    status_info : dict
        Dictionary to look through.
    keys : list
        List of keys to look through with nested dictionarie.
    default_value : str, optional
        A default value to return if the item value was not found.
    precision : int, optional
        Precision requested for the float values. Defaults to 4.
    format : str, optional
        Format specifier to use for the floating point value. Defaults to 'f'.
    scale : float, optional
        Scale to apply to value prior to formatting.
    include_plus_sign : bool, optional
        Include a plus sign before a positive value for text alignment
        purposes. Defaults to False.

    Returns
    -------
    value : dictionary item value, or `N/A`
        Value of the last key in the `keys` list formated with precision.
    """
    try:
        value = reduce(operator.getitem, keys, status_info)
        if isinstance(value, (int, float)):
            value = float(value) * scale
            if include_plus_sign:
                fmt = '{:+.%d%s}' % (precision, format)
            else:
                fmt = '{:.%d%s}' % (precision, format)
            return fmt.format(value)
        return value
    except KeyError:
        return default_value


def format_status_table(status_info, row_to_key, column_to_key,
                        row_identifier='Index'):
    """
    Create a PrettyTable based on status information.

    Parameters
    ----------
    status_info : dict
        The status information dictionary.

    row_to_key : dict
        Dictionary of the form ``{'display_text': 'status_key'}``.

    column_to_key : dict
        Dictionary of the form ``{'display_text': 'status_key'}``.

    row_identifier : str, optional
        What to show for the first column, likely an 'Index' or 'Component'.
    """

    table = prettytable.PrettyTable()
    table.field_names = [row_identifier] + list(column_to_key)
    for row_name, row_key in row_to_key.items():
        row = [
            get_status_value(status_info, row_key, key, 'value')
            for key in column_to_key.values()
        ]
        table.add_row([str(row_name)] + row)

    return table


def combine_status_info(obj, status_info, attrs, separator='\n= {attr} ='):
    """
    Combine status information from the given attributes.

    Parameters
    ----------
    obj : OphydObj
        The parent ophyd object.

    status_info : dict
        The status information dictionary.

    attrs : sequence of str
        The attribute names.

    separator : str, optional
        The separator line between the statuses.  May be formatted
        with variables ``attr``, ``parent``, or ``child``.
    """
    lines = []
    for attr in attrs:
        child = getattr(obj, attr)
        lines.append(separator.format(attr=attr, parent=obj, child=child))
        lines.append(child.format_status_info(status_info[attr]))

    return '\n'.join(lines)


def doc_format_decorator(**doc_fmts):
    """
    Decorator for substituting values into a docstring.

    This is useful for cases where we want to include module
    constants in docstrings but we want the docstring to
    update automatically when we decide to change the
    constant.
    """
    def inner_decorator(func):
        func.__doc__ = func.__doc__.format(**doc_fmts)
        return func
    return inner_decorator


EnumId = Union[enum.Enum, int, str]


class HelpfulIntEnumMeta(enum.EnumMeta):
    def __getattr__(self, key):
        if hasattr(key, "lower"):
            for item in self:
                if item.name.lower() == key.lower():
                    return item
        return super().__getattr__(key)

    def __getitem__(self, key):
        if hasattr(key, "lower"):
            for item in self:
                if item.name.lower() == key.lower():
                    return item
        return super().__getitem__(key)


class HelpfulIntEnum(enum.IntEnum, metaclass=HelpfulIntEnumMeta):
    """
    IntEnum subclass with some utility extensions and case insensitivity.
    """

    @classmethod
    def from_any(cls, identifier: EnumId) -> HelpfulIntEnum:
        """
        Try all the ways to interpret identifier as the enum.
        This is intended to consolidate the try/except tree typically used
        to interpret external input as an enum.

        Parameters
        ----------
        identifier : EnumId
            Any str, int, or Enum value that corresponds with a valid value
            on this HelpfulIntEnum instance.

        Returns
        -------
        enum : HelpfulIntEnum
            The corresponding enum object associated with the identifier.
        """
        try:
            return cls[identifier]
        except KeyError:
            return cls(identifier)

    @classmethod
    def include(
        cls,
        identifiers: Iterator[EnumId],
    ) -> set[HelpfulIntEnum]:
        """
        Returns all enum values matching the identifiers given.
        This is a shortcut for calling cls.from_any many times and
        assembling a set of the results.

        Parameters
        ----------
        identifiers : Iterator[EnumId]
            Any iterable that contains strings, ints, and Enum values that
            correspond with valid values on this HelpfulIntEnum instance.

        Returns
        -------
        enums : set[HelpfulIntEnum]
            A set whose elements are the enum objects associated with the
            input identifiers.
        """
        return {cls.from_any(ident) for ident in identifiers}

    @classmethod
    def exclude(
        cls,
        identifiers: Iterator[EnumId],
    ) -> set[HelpfulIntEnum]:
        """
        Return all enum values other than the ones given.

        Parameters
        ----------
        identifiers : Iterator[EnumId]
            Any iterable that contains strings, ints, and Enum values that
            correspond with valid values on this HelpfulIntEnum instance.

        Returns
        -------
        enums : set[HelpfulIntEnum]
            A set whose elements are the valid enum objects not associated
            with the input identifiers.
        """
        return set(cls.__members__.values()) - cls.include(identifiers)
