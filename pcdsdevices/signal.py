"""
Module to define ophyd Signal subclass utilities.
"""
# Catch semi-frequent issue with scripts accidentally run from inside module
if __name__ != 'pcdsdevices.signal':
    raise RuntimeError('A script tried to import pcdsdevices.signal '
                       'instead of the signal built-in module. This '
                       'usually happens when a script is run from '
                       'inside the pcdsdevices directory and can cause '
                       'extremely confusing bugs. Please run your script '
                       'elsewhere for better results.')
import itertools
import logging
import numbers
import typing
from threading import RLock
from typing import Optional

import numpy as np
import ophyd
from ophyd.signal import (DerivedSignal, EpicsSignal, EpicsSignalBase,
                          EpicsSignalRO, Signal, SignalRO)
from ophyd.sim import FakeEpicsSignal, FakeEpicsSignalRO, fake_device_cache
from pytmc.pragmas import normalize_io

from .utils import convert_unit

logger = logging.getLogger(__name__)


class PytmcSignal(EpicsSignalBase):
    """
    Class for a connection to a pytmc-generated EPICS record.

    This uses the same args as the pragma, so you can refer to the pytmc
    pragmas to select args for your components. This will automatically append
    the '_RBV' suffix and wrap the read/write PVs into the same signal object
    as appropriate, and pick between a read-only signal and a writable one.

    Under the hood this actually gives you the RW or RO version of the signal
    depending on your io argument.
    """

    def __new__(cls, prefix, io=None, **kwargs):
        new_cls = select_pytmc_class(io=io, prefix=prefix,
                                     write_cls=PytmcSignalRW,
                                     read_only_cls=PytmcSignalRO)
        return super().__new__(new_cls)

    def __init__(self, prefix, *, io, **kwargs):
        self.pytmc_pv = prefix
        self.pytmc_io = io
        super().__init__(prefix + '_RBV', **kwargs)


def select_pytmc_class(io=None, *, prefix, write_cls, read_only_cls):
    """Return the class to use for PytmcSignal's constructor."""
    if io is None:
        # Provide a better error here than "__new__ missing an arg"
        raise ValueError('Must provide an "io" argument to PytmcSignal. '
                         f'This is missing for signal with pv {prefix}. '
                         'Feel free to copy the io field from the '
                         'pytmc pragma.')
    if pytmc_writable(io):
        return write_cls
    else:
        return read_only_cls


def pytmc_writable(io):
    """Returns `True` if the pytmc io arg represents a writable PV."""
    norm = normalize_io(io)
    if norm == 'output':
        return True
    elif norm == 'input':
        return False
    else:
        # Should never get here unless pytmc's API changes
        raise ValueError(f'Invalid io specifier {io}')


class PytmcSignalRW(PytmcSignal, EpicsSignal):
    """Read-write connection to a pytmc-generated EPICS record."""
    def __init__(self, prefix, **kwargs):
        super().__init__(prefix, write_pv=prefix, **kwargs)


class PytmcSignalRO(PytmcSignal, EpicsSignalRO):
    """Read-only connection to a pytmc-generated EPICS record."""
    pass


# Make sure an acceptable fake class is set for PytmcSignal
class FakePytmcSignal(FakeEpicsSignal):
    """A suitable fake class for PytmcSignal."""
    def __new__(cls, prefix, io=None, **kwargs):
        new_cls = select_pytmc_class(io=io, prefix=prefix,
                                     write_cls=FakePytmcSignalRW,
                                     read_only_cls=FakePytmcSignalRO)
        return super().__new__(new_cls)

    def __init__(self, prefix, io=None, **kwargs):
        super().__init__(prefix + '_RBV', **kwargs)


class FakePytmcSignalRW(FakePytmcSignal, FakeEpicsSignal):
    def __init__(self, prefix, **kwargs):
        super().__init__(prefix, write_pv=prefix, **kwargs)


class FakePytmcSignalRO(FakePytmcSignal, FakeEpicsSignalRO):
    pass


# NOTE: This is an *on-import* update of the ophyd "fake" device cache
fake_device_cache[PytmcSignal] = FakePytmcSignal


class AggregateSignal(Signal):
    """
    Signal that is composed of a number of other signals.

    This class exists to handle the group subscriptions without repeatedly
    getting the values of all the subsignals at all times.

    Attributes
    ----------
    _cache : dict
        Mapping from signal to last known value.

    _sub_signals : list
        Signals that contribute to this signal.
    """

    _update_only_on_change = True

    def __init__(self, *, name, **kwargs):
        super().__init__(name=name, **kwargs)
        self._cache = {}
        self._has_subscribed = False
        self._lock = RLock()
        self._sub_signals = []

    def _calc_readback(self):
        """
        Override this with a calculation to find the current state given the
        cached values.

        Returns
        -------
        readback
            The result of the calculation.
        """

        raise NotImplementedError('Subclasses must implement _calc_readback')

    def _insert_value(self, signal, value):
        """Update the cache with one value and recalculate."""
        with self._lock:
            self._cache[signal] = value
            self._update_state()
            return self._readback

    def _update_state(self):
        """Recalculate the state."""
        with self._lock:
            self._readback = self._calc_readback()

    def get(self, **kwargs):
        """Update all values and recalculate."""
        with self._lock:
            for signal in self._sub_signals:
                self._cache[signal] = signal.get(**kwargs)
            self._update_state()
            return self._readback

    def put(self, value, **kwargs):
        raise NotImplementedError('put should be overriden in the subclass')

    def subscribe(self, cb, event_type=None, run=True):
        """
        Set up a callback function to run at specific times.

        See the `ophyd` documentation for details.
        """

        cid = super().subscribe(cb, event_type=event_type, run=run)
        if event_type in (None, self.SUB_VALUE) and not self._has_subscribed:
            # We need to subscribe to ALL relevant signals!
            for signal in self._sub_signals:
                signal.subscribe(self._run_sub_value, run=False)
            self.get()  # Ensure we have a full cache
        return cid

    def _run_sub_value(self, *args, **kwargs):
        kwargs.pop('sub_type')
        sig = kwargs.pop('obj')
        kwargs.pop('old_value')
        value = kwargs['value']
        with self._lock:
            old_value = self._readback
            # Update just one value and assume the rest are cached
            # This allows us to run subs without EPICS gets
            value = self._insert_value(sig, value)
            if value != old_value or not self._update_only_on_change:
                self._run_subs(sub_type=self.SUB_VALUE, obj=self, value=value,
                               old_value=old_value)


class PVStateSignal(AggregateSignal):
    """
    Signal that implements the `PVStatePositioner` state logic.

    See `AggregateSignal` for more information.
    """

    def __init__(self, *, name, **kwargs):
        super().__init__(name=name, **kwargs)
        self._sub_map = {}
        for signal_name in self.parent._state_logic.keys():
            sig = self.parent
            for part in signal_name.split('.'):
                sig = getattr(sig, part)
            self._sub_signals.append(sig)
            self._sub_map[signal_name] = sig

    def describe(self):
        # Base description information
        sub_sigs = [sig.name for sig in self._sub_signals]
        desc = {'source': 'SUM:{}'.format(','.join(sub_sigs)),
                'dtype': 'string',
                'shape': [],
                'enum_strs': tuple(state.name
                                   for state in self.parent.states_enum)}
        return {self.name: desc}

    def _calc_readback(self):
        state_value = None
        for signal_name, info in self.parent._state_logic.items():
            # Get last cached value
            value = self._cache[self._sub_map[signal_name]]
            try:
                signal_state = info[value]
            # Handle unaccounted readbacks
            except KeyError:
                state_value = self.parent._unknown
                break
            # Associate readback with device state
            if signal_state != 'defer':
                if state_value:
                    # Handle inconsistent readbacks
                    if signal_state != state_value:
                        state_value = self.parent._unknown
                        break
                else:
                    # Set state to first non-deferred value
                    state_value = signal_state
                    if self.parent._state_logic_mode == 'ALL':
                        continue
                    elif self.parent._state_logic_mode == 'FIRST':
                        break
        # If all states deferred, report as unknown
        return state_value or self.parent._unknown

    def put(self, value, **kwargs):
        self.parent.move(value, **kwargs)


class AvgSignal(Signal):
    """
    Signal that acts as a rolling average of another signal.

    This will subscribe to a signal, and fill an internal buffer with values
    from `SUB_VALUE`. It will update its own value to be the mean of the last n
    accumulated values, up to the buffer size. If we haven't filled this
    buffer, this will still report a mean value composed of all the values
    we've receieved so far.

    Warning: this means that if we only have recieved ONE value, the mean will
    just be the mean of a single value!

    Parameters
    ----------
    signal : Signal
        Any subclass of `ophyd.signal.Signal` that returns a numeric value.
        This signal will be subscribed to be `AvgSignal` to calculate the mean.

    averages : int
        The number of `SUB_VALUE` updates to include in the average. New values
        after this number is reached will begin overriding old values.
    """

    def __init__(self, signal, averages, *, name, parent=None, **kwargs):
        super().__init__(name=name, parent=parent, **kwargs)
        if isinstance(signal, str):
            signal = getattr(parent, signal)
        self.raw_sig = signal
        self._lock = RLock()
        self.averages = averages
        self.raw_sig.subscribe(self._update_avg)

    @property
    def connected(self):
        return self.raw_sig.connected

    @property
    def averages(self):
        """The size of the internal buffer of values to average over."""
        return self._avg

    @averages.setter
    def averages(self, avg):
        """Reinitialize an empty internal buffer of size `avg`."""
        with self._lock:
            self._avg = avg
            self.index = 0
            # Allocate uninitalized array
            self.values = np.empty(avg)
            # Fill with nan
            self.values.fill(np.nan)

    def _update_avg(self, *args, value, **kwargs):
        """Add new value to the buffer, overriding old values if needed."""
        with self._lock:
            self.values[self.index] = value
            self.index = (self.index + 1) % len(self.values)
            # This takes a mean, skipping nan values.
            self.put(np.nanmean(self.values))


class NotImplementedSignal(SignalRO):
    """Dummy signal for a not implemented feature."""

    def __init__(self, *args, **kwargs):
        kwargs.pop('value', None)
        super().__init__(value='Not implemented', **kwargs)


class InternalSignal(SignalRO):
    """
    Class Signal that stores info but should only be updated by the class.

    SignalRO can be updated with _readback, but this does not process
    callbacks. For the signal to behave normally, we need to bypass the put
    override.

    To put to one of these signals, simply call put with force=True
    """

    def put(self, value, *, timestamp=None, force=False):
        return Signal.put(self, value, timestamp=timestamp, force=force)

    def set(self, value, *, timestamp=None, force=False):
        return Signal.set(self, value, timestamp=timestamp, force=force)


class _OptionalEpicsSignal(Signal):
    """
    An EPICS Signal which may or may not exist.

    The init parameters mirror those of :class:`~ophyd.EpicsSignal`.

    Notes
    -----
    This should be considered for internal use only, and not for
    user-facing device components.  If you use this in your new device,
    there is a good chance we will reject your PR.
    """

    def __init__(self, read_pv, write_pv=None, *, name, parent=None, kind=None,
                 **kwargs):
        self._saw_connection = False
        self._epics_signal = EpicsSignal(
            read_pv=read_pv, write_pv=write_pv, parent=self, name=name,
            kind=kind, **kwargs
        )
        super().__init__(name=name, parent=parent, kind=kind)
        self._epics_signal.subscribe(
            self._epics_meta_update,
            event_type=self._epics_signal.SUB_META,
        )

    def _epics_value_update(self, **kwargs):
        """The EpicsSignal value updated."""
        super().put(value=kwargs['value'], timestamp=kwargs['timestamp'],
                    force=True)
        # Note: the above internally calls run_subs
        # self._run_subs(**kwargs)

    def _epics_meta_update(self, sub_type=None, **kwargs):
        """The EpicsSignal metadata updated; reflect that here."""
        self._metadata.update(**kwargs)
        self._run_subs(sub_type=self.SUB_META, **kwargs)

        if not self._saw_connection and kwargs.get('connected', False):
            self._epics_signal.subscribe(self._epics_value_update)
            self._saw_connection = True

    def destroy(self):
        super().destroy()
        self._epics_signal.destroy()
        self._epics_signal = None

    def should_use_epics_signal(self) -> bool:
        """
        Tell `_OptionalEpicsSignal` whether or not to use the `EpicsSignal`.

        By default, the `EpicsSignal` will be used if the PV has connected.

        Note
        ----
        * Subclasses should override this with their own functionality.
        * This value should not change during the lifetime of the
          `_OptionalEpicsSignal`.
        """
        return self._saw_connection

    def _proxy_method(method_name):  # noqa
        """
        Proxy a method from either the EpicsSignal or the superclass Signal.
        """

        def method_selector(self, *args, **kwargs):
            owner = (self._epics_signal if self.should_use_epics_signal()
                     else super())
            return getattr(owner, method_name)(*args, **kwargs)

        return method_selector

    describe = _proxy_method('describe')
    describe_configuration = _proxy_method('describe_configuration')
    get = _proxy_method('get')
    put = _proxy_method('put')
    set = _proxy_method('set')
    read = _proxy_method('read')
    read_configuration = _proxy_method('read_configuration')
    wait_for_connection = _proxy_method('wait_for_connection')

    def _proxy_property(prop_name, value):  # noqa
        """Read-only property proxy for the internal EPICS Signal."""
        def getter(self):
            if self.should_use_epics_signal():
                return getattr(self._epics_signal, prop_name)
            return value

        # Only support read-only properties for now.
        return property(getter)

    connected = _proxy_property('connected', True)
    read_access = _proxy_property('read_access', True)
    write_access = _proxy_property('write_access', True)
    precision = _proxy_property('precision', 4)
    enum_strs = _proxy_property('enum_strs', ())
    limits = _proxy_property('limits', (0, 0))

    @property
    def kind(self):
        """The EPICS signal's kind."""
        return self._epics_signal.kind

    @kind.setter
    def kind(self, value):
        self._epics_signal.kind = value


class NotepadLinkedSignal(_OptionalEpicsSignal):
    """
    Create the notepad metadata dict for usage by pcdsdevices-notepad.
    For further information, see :class:`NotepadLinkedSignal`.

    Parameters
    ----------
    read_pv : str
        The PV to read from.

    write_pv : str, optional
        The PV to write to if different from the read PV.

    notepad_metadata : dict
        Base metadata for the notepad IOC.  This is a required keyword-only
        argument.  May include keys ``{"record_type", "default_value"}``.

    Note
    ----
    Arguments ``attr_name``, ``parent``, and ``name`` are passed in
    automatically by the ophyd Device machinery and do not need to be specified
    here.

    See also
    --------
    For further argument information, see :class:`~ophyd.EpicsSignal`.
    """

    @staticmethod
    def create_notepad_metadata(
            base_metadata, dotted_name, read_pv, write_pv=None, *,
            attr_name=None, parent=None, name=None, **kwargs):
        """
        Create the notepad metadata dict for usage by pcdsdevices-notepad.
        For further information, see :class:`NotepadLinkedSignal`.
        """
        return dict(
            **base_metadata,
            read_pv=read_pv,
            write_pv=write_pv,
            name=name,
            owner_type=type(parent).__name__,
            dotted_name=dotted_name,
            signal_kwargs={key: value
                           for key, value in kwargs.items()
                           if isinstance(value, (int, str, float))
                           },
        )

    def __init__(self, read_pv, write_pv=None, *, notepad_metadata,
                 attr_name=None, parent=None, name=None, **kwargs):
        # Pre-define some attributes so we can aggregate information:
        self._parent = parent
        self._attr_name = attr_name
        self._name = name
        if self.root is self:
            full_dotted_name = attr_name
        else:
            full_dotted_name = f'{self.root.name}.{attr_name}'

        self.notepad_metadata = self.create_notepad_metadata(
            base_metadata=notepad_metadata,
            dotted_name=full_dotted_name,
            read_pv=read_pv, write_pv=write_pv, name=name, parent=parent,
            **kwargs
        )
        super().__init__(read_pv=read_pv, write_pv=write_pv, parent=parent,
                         attr_name=attr_name, name=name, **kwargs)


class FakeNotepadLinkedSignal(FakeEpicsSignal):
    """A suitable fake class for NotepadLinkedSignal."""
    def __init__(self, read_pv, write_pv=None, *, notepad_metadata,
                 attr_name=None, parent=None, name=None,
                 **kwargs):
        # Pre-define some attributes so we can aggregate information:
        self._parent = parent
        self._attr_name = attr_name
        self.notepad_metadata = NotepadLinkedSignal.create_notepad_metadata(
            base_metadata=notepad_metadata,
            dotted_name=self.root.name + '.' + self.dotted_name,
            read_pv=read_pv, write_pv=write_pv, name=name, parent=parent,
            **kwargs
        )
        super().__init__(read_pv=read_pv, write_pv=write_pv, parent=parent,
                         attr_name=attr_name, name=name, **kwargs)


# NOTE: This is an *on-import* update of the ophyd "fake" device cache
fake_device_cache[NotepadLinkedSignal] = FakeNotepadLinkedSignal


class UnitConversionDerivedSignal(DerivedSignal):
    """
    A DerivedSignal which performs unit conversion.

    Custom units may be specified for the original signal, or if specified, the
    original signal's units may be retrieved upon first connection.

    Parameters
    ----------
    derived_from : Signal or str
        The signal from which this one is derived.  This may be a string
        attribute name that indicates a sibling to use.  When used in a
        ``Device``, this is then simply the attribute name of another
        ``Component``.

    derived_units : str
        The desired units to use for this signal.  These can also be referred
        to as the "user-facing" units.

    original_units : str, optional
        The units from the original signal.  If not specified, control system
        information regarding units will be retrieved upon first connection.

    user_offset : any, optional
        An optional user offset that will be *subtracted* when updating the
        original signal, and *added* when calculating the derived value.
        This offset should be supplied in ``derived_units`` and not
        ``original_units``.

        For example, if the original signal updates to a converted value of
        500 ``derived_units`` and the ``user_offset`` is set to 100, this
        ``DerivedSignal`` will show a value of 600.  When providing a new
        setpoint, the ``user_offset`` will be subtracted.

    write_access : bool, optional
        Write access may be disabled by setting this to ``False``, regardless
        of the write access of the underlying signal.

    name : str, optional
        The signal name.

    parent : Device, optional
        The parent device.  Required if ``derived_from`` is an attribute name.

    limits : 2-tuple, optional
        Ophyd signal-level limits in derived units.  DerivedSignal defaults
        to converting the original signal's limits, but these may be overridden
        here without modifying the original signal.

    **kwargs :
        Keyword arguments are passed to the superclass.
    """

    derived_units: str
    original_units: str

    def __init__(self, derived_from, *,
                 derived_units: str,
                 original_units: typing.Optional[str] = None,
                 user_offset: typing.Optional[numbers.Real] = 0,
                 limits: typing.Optional[typing.Tuple[numbers.Real,
                                                      numbers.Real]] = None,
                 **kwargs):
        self.derived_units = derived_units
        self.original_units = original_units
        self._user_offset = user_offset
        self._custom_limits = limits
        super().__init__(derived_from, **kwargs)
        self._metadata['units'] = derived_units

        # Ensure that we include units in metadata callbacks, even if the
        # original signal does not include them.
        if 'units' not in self._metadata_keys:
            self._metadata_keys = self._metadata_keys + ('units', )

    def forward(self, value):
        '''Compute derived signal value -> original signal value'''
        if self.user_offset is None:
            raise ValueError(f'{self.name} must be set to a non-None value.')
        return convert_unit(value - self.user_offset,
                            self.derived_units, self.original_units)

    def inverse(self, value):
        '''Compute original signal value -> derived signal value'''
        if self.user_offset is None:
            raise ValueError(f'{self.name} must be set to a non-None value.')
        return convert_unit(value, self.original_units,
                            self.derived_units) + self.user_offset

    @property
    def limits(self):
        '''
        Defaults to limits from the original signal (low, high).

        Limit values may be reversed such that ``low <= value <= high`` after
        performing the calculation.

        Limits may also be overridden here without affecting the original
        signal.
        '''
        if self._custom_limits is not None:
            return self._custom_limits

        # Fall back to the superclass derived_from limits:
        return tuple(
            sorted(self.inverse(v) for v in self._derived_from.limits)
        )

    @limits.setter
    def limits(self, value):
        if value is None:
            self._custom_limits = None
            return

        if len(value) != 2 or value[0] >= value[1]:
            raise ValueError('Custom limits must be a 2-tuple (low, high)')

        self._custom_limits = tuple(value)

    @property
    def user_offset(self) -> typing.Optional[typing.Any]:
        """A user-specified offset in *derived*, user-facing units."""
        return self._user_offset

    @user_offset.setter
    def user_offset(self, offset):
        offset_change = -self._user_offset + offset
        self._user_offset = offset
        self._recalculate_position()
        if self._custom_limits is not None:
            self._custom_limits = (
                self._custom_limits[0] + offset_change,
                self._custom_limits[1] + offset_change,
            )

    def _recalculate_position(self):
        """
        Recalculate the derived position and send subscription updates.

        No-operation if the original signal is not connected.
        """
        if not self._derived_from.connected:
            return

        value = self._derived_from.get()
        if value is not None:
            # Note: no kwargs here; no metadata updates
            self._derived_value_callback(value)

    def _derived_metadata_callback(self, *, connected, **kwargs):
        if connected and 'units' in kwargs:
            if self.original_units is None:
                self.original_units = kwargs['units']
        # Do not pass through units, as we have our own.
        kwargs['units'] = self.derived_units
        super()._derived_metadata_callback(connected=connected, **kwargs)

    def describe(self):
        full_desc = super().describe()
        desc = full_desc[self.name]
        desc['units'] = self.derived_units
        # Note: this should be handled in ophyd:
        for key in ('lower_ctrl_limit', 'upper_ctrl_limit'):
            if key in desc:
                desc[key] = self.inverse(desc[key])
        return full_desc


class SignalEditMD(Signal):
    """
    Subclass for allowing an external override of signal metadata.

    This can be useful in cases where the signal metadata is wrong, not
    included properly in the signal, or where you'd like different
    metadata-dependent behavior than what is discovered via the cl.

    Does some minimal checking against the signal's metadata keys and ensures
    the override values always take priority over the normally found values.
    """
    def _override_metadata(self, **md):
        """
        Externally override the signal metadata.

        This is a semi-private member that should only be called from device
        classes on their own signals.
        """
        for key in md.keys():
            if key not in self._metadata_keys:
                raise ValueError(
                    f'Tried to override metadata key {key} in {self.name}, '
                    'but this is not one of the metadata keys: '
                    f'{self._metadata_keys}'
                    )
        try:
            self._metadata_override.update(**md)
        except AttributeError:
            self._metadata_override = md
        self._run_metadata_callbacks()

    @property
    def metadata(self):
        md = {}
        md.update(self._metadata)
        try:
            md.update(self._metadata_override)
        except AttributeError:
            pass
        return md

    # Switch out _metadata for metadata
    def _run_metadata_callbacks(self):
        self._metadata_thread_ctx.run(self._run_subs, sub_type=self.SUB_META,
                                      **self.metadata)


class EpicsSignalBaseEditMD(EpicsSignalBase, SignalEditMD):
    """
    EpicsSignal variant which allows for user correction of various metadata.

    Parameters
    ----------
    enum_strings : list of str, optional
        List of enum strings to replace the EPICS originals.  May not be
        used in conjunction with the dynamic ``enum_attrs``.

    enum_attrs : list of str, optional
        List of signal attribute names, relative to the parent device.  That is
        to say a given attribute is assumed to be a sibling of this signal
        instance.  Attribute names may be ``None`` in the case where the
        original enum string should be passed through.

    See Also
    ---------
    `ophyd.signal.EpicsSignal` for further parameter information.
    """
    _enum_attrs: list[Optional[str]]
    _enum_count: int
    _enum_strings: list[str]
    _original_enum_strings: list[str]
    _enum_signals: list[Optional[ophyd.ophydobj.OphydObject]]
    _enum_string_override: bool
    _enum_subscriptions: dict[ophyd.ophydobj.OphydObject, int]
    _pending_signals: set[ophyd.ophydobj.OphydObject]
    _sent_first_md_callbacks: bool

    def __init__(
        self,
        *args,
        enum_attrs: Optional[list[Optional[str]]] = None,
        enum_strs: Optional[list[str]] = None,
        **kwargs
    ):
        super().__init__(*args, **kwargs)

        self._enum_attrs = list(enum_attrs or [])
        self._pending_signals = set()
        self._original_enum_strings = []
        self._enum_signals = []
        self._enum_subscriptions = {}
        self._enum_count = 0
        self._metadata_override = {}
        self._sent_first_md_callbacks = False

        if enum_attrs and enum_strs:
            raise ValueError(
                "enum_attrs OR enum_strs may be set, but not both"
            )

        self._enum_string_override = bool(enum_attrs or enum_strs)
        if self._enum_string_override:
            # We need to control 'connected' status based on other signals
            self._metadata_override["connected"] = False

        if enum_attrs:
            # Override by way of other signals
            self._enum_strings = [""] * len(self.enum_attrs)
            # The following magic is provided by EpicsSignalBaseEditMD.
            # The end result is:
            # -> self.metadata["enum_strs"] => self._enum_strings
            self._metadata_override["enum_strs"] = self._enum_strings
            if self.parent is None:
                raise RuntimeError(
                    "This signal {self.name!r} must be used in a "
                    "Device/Component hierarchy."
                )

            self._subscribe_enum_attrs()

        elif enum_strs:
            # Override with strings
            self._enum_strings = list(enum_strs)
            self._metadata_override["enum_strs"] = self._enum_strings

    def destroy(self):
        super().destroy()
        for sig, sub in self._enum_subscriptions.items():
            if sig is not None:
                sig.unsubscribe(sub)
        self._enum_subscriptions.clear()

    def _subscribe_enum_attrs(self):
        """Subscribe to enum signals by attribute name."""
        for attr in self.enum_attrs:
            if attr is None:
                # Opt-out for a specific signal
                self._enum_signals.append(None)
                continue

            try:
                obj = getattr(self.parent, attr)
            except AttributeError as ex:
                raise RuntimeError(
                    f"Attribute {attr!r} specified in enum list appears to be "
                    f"invalid for the device {self.parent.name}."
                ) from ex

            if obj is self:
                raise RuntimeError(
                    f"Recursively specified {self.name!r} in the enum_attrs "
                    "list.  Don't do that."
                )
            self._enum_signals.append(obj)
            self._pending_signals.add(obj)
            self._enum_subscriptions[obj] = obj.subscribe(
                self._enum_string_updated, run=True
            )

    # Switch out _metadata for metadata where appropriate
    @property
    def enum_strs(self) -> list[str]:
        """
        List of enum strings.

        For an EpicsSignalEditMD, this could be one of:

        1. The original enum strings from the PV
        2. The strings found from the respective signals referenced by
            ``enum_attrs``.
        3. The user-provided strings in ``enum_strs``.
        """
        if self._enum_string_override:
            return list(self._enum_strings)[:self._enum_count]
        return self.metadata['enum_strs']

    @property
    def precision(self):
        """The PV precision as reported by EPICS (or EpicsSignalEditMD)."""
        return self.metadata['precision']

    @property
    def limits(self) -> tuple[numbers.Real, numbers.Real]:
        """The PV limits as reported by EPICS (or EpicsSignalEditMD)."""
        return (self.metadata['lower_ctrl_limit'],
                self.metadata['upper_ctrl_limit'])

    def describe(self):
        """
        Return the signal description as a dictionary.

        Units, limits, precision, and enum strings may be overridden.

        Returns
        -------
        dict
            Dictionary of name and formatted description string
        """
        desc = super().describe()
        desc[self.name]['units'] = self.metadata['units']
        return desc

    @property
    def enum_attrs(self) -> list[str]:
        """Enum attribute names - the source of each enum string."""
        return list(self._enum_attrs)

    def _enum_string_updated(
        self,
        value: str,
        obj: ophyd.ophydobj.OphydObject,
        **kwargs
    ):
        """
        A single Signal from ``enum_signals`` updated its value.

        This is a ``SUB_VALUE`` subscription callback from that signal.

        Parameters
        ----------
        value : str
            The value of that enum index.

        obj : ophyd.ophydobj.OphydObject
            The ophyd object with the value.

        **kwargs :
            Additional metadata from ``self._metadata``.
        """
        if value is None:
            # The callback may run before it's connected
            return

        try:
            idx = self._enum_signals.index(obj)
        except IndexError:
            return

        if value == "Invalid":
            try:
                value = self._original_enum_strings[idx]
            except IndexError:
                ...

        self._enum_strings[idx] = str(value)

        self.log.debug(
            "Got enum %s [%d] = %s from %s",
            self.name, idx, value, getattr(obj, "pvname", "(no pvname)")
        )
        try:
            self._pending_signals.remove(obj)
        except KeyError:
            ...

        if not self._pending_signals:
            # We're probably connected!
            self._run_metadata_callbacks()

    @property
    def connected(self) -> bool:
        """Is the signal connected and ready to use?"""
        return (
            self._metadata["connected"]
            and not self._destroyed
            and not len(self._pending_signals)
        )

    def _check_signal_metadata(self):
        """Check the original enum strings to compare the attributes."""
        if not self._enum_string_override:
            return

        self._original_enum_strings = self._metadata.get(
            "enum_strs", None
        ) or []
        if not self._original_enum_strings:
            self.log.error(
                "No enum strings on %r; was %r used inappropriately?",
                self.pvname, type(self).__name__
            )
            return

        if self._enum_count == 0:
            self._enum_count = len(self._original_enum_strings)

            def pick_enum_string(existing: str, original: str) -> str:
                """
                Pick the best enum string, given two options.

                The "Invalid" marker indicates that the PLC sName for the state
                has not yet been updated.  Ignore it and fall back to the
                PLC enum-defined value.
                """
                if existing.lower() == "invalid":
                    existing = ""
                return existing or original

            # Only update ones that have yet to be populated;  this can
            # be a race for who connects first:
            updated_enums = [
                pick_enum_string(existing, original)
                for existing, original in itertools.zip_longest(
                    self._enum_strings,
                    self._original_enum_strings,
                    fillvalue=""
                )
            ]
            self._enum_strings[:] = updated_enums

    def _run_metadata_callbacks(self):
        """Hook for metadata callbacks, mostly run by superclasses."""
        self._metadata_override["connected"] = self.connected
        # TODO: to truncate the number of enum strings reported to the number
        # that EPICS reports for the given PV, the following may be advisable.
        # However, it seems to add additional confusing errors; so consider
        # this a todo.
        # self._metadata_override["enum_strs"] = self.enum_strs
        if self._metadata["connected"]:
            # The underlying PV has connected - check its enum_strs:
            self._check_signal_metadata()
        if self.connected or self._sent_first_md_callbacks:
            self._sent_first_md_callbacks = True
            super()._run_metadata_callbacks()


class EpicsSignalEditMD(EpicsSignal, EpicsSignalBaseEditMD):
    pass


class EpicsSignalROEditMD(EpicsSignalRO, EpicsSignalBaseEditMD):
    pass


EpicsSignalEditMD.__doc__ = EpicsSignalBaseEditMD.__doc__ + EpicsSignal.__doc__
EpicsSignalROEditMD.__doc__ = (
    EpicsSignalBaseEditMD.__doc__ + EpicsSignalRO.__doc__
)


class FakeEpicsSignalEditMD(FakeEpicsSignal):
    """
    API stand-in for EpicsSignalEditMD
    Add to this if you need it to actually work for your test.
    """
    def __init__(
        self,
        *args,
        enum_attrs: Optional[list[Optional[str]]] = None,
        enum_strs: Optional[list[str]] = None,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self._enum_attrs = enum_attrs
        self._enum_strs = enum_strs

    @property
    def enum_attrs(self):
        return self._enum_attrs

    @property
    def enum_strs(self):
        return self._enum_attrs or self._enum_strs or None

    def _override_metadata(self, **kwargs):
        ...


class FakeEpicsSignalROEditMD(FakeEpicsSignalRO):
    """
    API stand-in for EpicsSignalROEditMD
    Add to this if you need it to actually work for your test.
    """
    def __init__(
        self,
        *args,
        enum_attrs: Optional[list[Optional[str]]] = None,
        enum_strs: Optional[list[str]] = None,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self._enum_attrs = enum_attrs
        self._enum_strs = enum_strs

    @property
    def enum_attrs(self):
        return self._enum_attrs

    @property
    def enum_strs(self):
        return self._enum_attrs or self._enum_strs or None

    def _override_metadata(self, **kwargs):
        ...


fake_device_cache[EpicsSignalEditMD] = FakeEpicsSignalEditMD
fake_device_cache[EpicsSignalROEditMD] = FakeEpicsSignalROEditMD
