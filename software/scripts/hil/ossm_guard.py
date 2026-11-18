"""OSSM motor interlock: no OSSM firmware runs unless the RS-485 guard protects it.

The independent guard (C:\\rad-hil\\rs485-guard, host-only) samples the 57AIM30
motor over RS-485 and owns the verified stop. Homing needs the motor handed back
to the OSSM, where the guard enforces its collision-only contact limits.

Pull-request runs take precedence over the bench's own state. When the guard
is missing, latched or not handed back, the run asks it to recover: the guard
(started by the rig watchdog if needed) checks the motor is idle, healthy and
between its measured stops, moves it slowly to mid-rail and hands it back. The
fault it cleared is kept in the evidence and shown on the pull request. If the
guard refuses or cannot recover, the run fails before anything is written.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

MAX_STATUS_AGE_S = 5.0
MIN_RATE_HZ = 30.0
MAX_SAMPLE_AGE_MS = 500.0
# A recovery moves the carriage up to the rail's length at 20 mm/s, then settles.
RECOVERY_WAIT_S = 180.0
RECOVERY_SETTLE_S = 20.0


def read_status(path):
    path = Path(path)
    # Windows can deny an open while the guard atomically replaces the file.
    # Limit the wait to 50 ms and check the returned sample's age as usual.
    for attempt in range(3):
        try:
            raw = path.read_text(encoding='utf-8-sig')
            break
        except PermissionError:
            if attempt == 2:
                raise
            time.sleep(0.025)
    status = json.loads(raw)
    written = datetime.fromisoformat(status['time'])
    status['status_age_s'] = (datetime.now(timezone.utc) - written).total_seconds()
    return status


def unsafe_reason(status):
    """Return why the guard cannot protect motion, or None when it can."""
    rail = status.get('rail') or {}
    if status['status_age_s'] > MAX_STATUS_AGE_S:
        return f"guard status is {status['status_age_s']:.0f} s old; the guard is not running"
    if not status.get('streamConnected'):
        return 'guard is not receiving bench telemetry'
    if (status.get('rateHz') or 0) < MIN_RATE_HZ:
        return f"RS-485 telemetry is {status.get('rateHz') or 0:.0f} Hz, below {MIN_RATE_HZ:.0f} Hz"
    if status.get('sampleAgeMs') is None or status['sampleAgeMs'] > MAX_SAMPLE_AGE_MS:
        return 'RS-485 telemetry is stale'
    if status.get('activeFault'):
        return f"guard fault: {status['activeFault']}"
    if not any(r.get('stopVerified') for r in status.get('stopResults') or []):
        return 'the guard has not verified its stop path'
    if rail.get('critical'):
        return 'the guard has latched the motor off; review the fault and hand the motor back before testing'
    if status.get('mode') != 'collision-only' or not (status.get('handback') or {}).get('verified'):
        return 'the motor has not been verifiably handed back to the OSSM under the guard'
    if (status.get('alerts') or {}) and not status.get('safe'):
        return 'guard alerts: ' + ', '.join(sorted(status['alerts']))
    return None


class GuardWatch:
    """Preflight plus continuous checks while an OSSM run is in progress."""

    def __init__(self, path, output, request_path=None, requester=''):
        self.path = Path(path)
        self.output = Path(output)
        # Where a recovery request is left for the guard; None never asks.
        self.request_path = Path(request_path) if request_path else None
        self.requester = requester
        self.failure = None
        self.record = dict(schema=1, preflight=None, motion=None, before=None, after=None,
                           faults=[], contacts=[], recovery=None)
        self._stop = threading.Event()
        self._thread = None
        self._faults = None

    def _inspect(self):
        """Return (status, reason); status is None when the guard left no readable status."""
        try:
            status = read_status(self.path)
        except (OSError, ValueError, KeyError) as error:
            return None, f'guard status unavailable: {error}'
        return status, unsafe_reason(status)

    def _recover(self, reason, quiet, wait_s, settle_s):
        """Ask the guard to clear its own state for this run. Returns the final (status, reason)."""
        request_id = f'{self.requester}@{time.time():.3f}'
        recovery = dict(requested=reason, id=request_id, outcome='no answer from the guard')
        self.record['recovery'] = recovery
        if quiet:
            # Stored STEP pulses run when the motor is handed back: silence the OSSM first.
            quiet()
        self.request_path.parent.mkdir(parents=True, exist_ok=True)
        self.request_path.write_text(json.dumps(dict(id=request_id, by=self.requester, time=time.time())),
                                     encoding='utf-8')
        try:
            deadline = time.monotonic() + wait_s
            answer = None
            while time.monotonic() < deadline:
                status, _ = self._inspect()
                answer = (status or {}).get('recovery') or {}
                if answer.get('id') == request_id and answer.get('outcome') in ('cleared', 'not-needed', 'failed'):
                    recovery.update(outcome=answer['outcome'], clearedFault=answer.get('clearedFault'),
                                    error=answer.get('error'), position=answer.get('position'))
                    break
                time.sleep(0.5)
            # The guard's rate window refills after a restart or a recovery.
            deadline = time.monotonic() + settle_s
            while True:
                status, reason = self._inspect()
                if not reason or time.monotonic() >= deadline:
                    return status, reason
                time.sleep(0.5)
        finally:
            try:
                self.request_path.unlink()
            except OSError:
                pass

    def require(self, quiet=None, wait_s=RECOVERY_WAIT_S, settle_s=RECOVERY_SETTLE_S):
        status, reason = self._inspect()
        if reason and self.request_path:
            status, reason = self._recover(reason, quiet, wait_s, settle_s)
        recovery = self.record['recovery']
        note = ''
        if recovery:
            note = f"; recovered automatically before this run from: {recovery['requested']}"
        if status is None:
            self.record['preflight'] = reason
            raise RuntimeError('RS-485 safety guard is not running; refusing to flash the OSSM')
        self.record['before'] = status
        if reason:
            detail = ''
            if recovery:
                detail = f" (automatic recovery: {recovery['outcome']}" + (
                    f", {recovery['error']}" if recovery.get('error') else '') + ')'
            self.record['preflight'] = reason + detail
            raise RuntimeError('RS-485 safety guard: ' + reason + detail)
        self.record['preflight'] = f"guard running at {status['rateHz']:.0f} Hz with a verified stop path" + note
        self.record['motion'] = 'handed back to the OSSM; guard enforcing collision-only limits'
        self._faults = status.get('faultCount', 0)
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def _watch(self):
        while not self._stop.wait(1.0):
            try:
                status = read_status(self.path)
            except (OSError, ValueError, KeyError) as error:
                self._fail(f'guard status unreadable: {error}')
                continue
            for contact in status.get('contacts') or []:
                if len(self.record['contacts']) < 10 and contact not in self.record['contacts']:
                    self.record['contacts'].append(contact)
            if status.get('faultCount', 0) != self._faults or status.get('activeFault'):
                self._faults = status.get('faultCount', 0)
                self.record['faults'].append(dict(time=status.get('time'), reason=status.get('activeFault')))
                self._fail(f"guard stopped the motor: {status.get('activeFault') or 'fault recorded'}")
            elif status['status_age_s'] > MAX_STATUS_AGE_S:
                self._fail('guard stopped reporting during the run')
            elif (status.get('rail') or {}).get('critical'):
                self._fail('guard latched the motor off during the run')

    def _fail(self, reason):
        if not self.failure:
            self.failure = 'RS-485 safety guard: ' + reason

    def check(self):
        if self.failure:
            raise RuntimeError(self.failure)

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        try:
            self.record['after'] = read_status(self.path)
        except (OSError, ValueError, KeyError) as error:
            self.record['after'] = dict(error=str(error))
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / 'rs485-guard.json').write_text(json.dumps(self.record, indent=2, default=str) + '\n',
                                                      encoding='utf-8')
