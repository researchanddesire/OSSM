"""Video evidence never decides a result; the OSSM guard interlock always can."""
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ossm_guard import GuardWatch, read_status, unsafe_reason  # noqa: E402
from video import Recorder, view_filter  # noqa: E402


def guard_status(**changes):
    status = dict(time=datetime.now(timezone.utc).isoformat(), streamConnected=True, rateHz=60.0,
                  sampleAgeMs=4.0, activeFault=None, faultCount=0, safe=True, alerts={},
                  stopResults=[dict(stopVerified=True)], rail=dict(enabled=True, critical=False),
                  mode='collision-only', handback=dict(verified=True), contacts=[])
    status.update(changes)
    return status


class GuardTests(unittest.TestCase):
    def reason(self, **changes):
        status = guard_status(**changes)
        status['status_age_s'] = (datetime.now(timezone.utc) - datetime.fromisoformat(status['time'])).total_seconds()
        return unsafe_reason(status)

    def test_handed_back_guard_permits_the_run(self):
        self.assertIsNone(self.reason())

    def test_every_unprotected_state_blocks_the_run(self):
        old = (datetime.now(timezone.utc) - timedelta(seconds=30)).isoformat()
        for changes in (dict(time=old), dict(streamConnected=False), dict(rateHz=12.0), dict(sampleAgeMs=None),
                        dict(sampleAgeMs=900.0), dict(activeFault='stall'), dict(stopResults=[]),
                        dict(rail=dict(critical=dict(reason='trip'))), dict(mode='strict'),
                        dict(handback=None), dict(safe=False, alerts=dict(supply_out_of_range={}))):
            with self.subTest(changes=changes):
                self.assertIsNotNone(self.reason(**changes))

    def test_missing_guard_refuses_before_any_write(self):
        with tempfile.TemporaryDirectory() as root:
            watch = GuardWatch(Path(root) / 'missing.json', root)
            with self.assertRaises(RuntimeError):
                watch.require()
            watch.close()
            self.assertIn('unavailable', json.loads((Path(root) / 'rs485-guard.json').read_text())['preflight'])

    def recover(self, before, answer, after, **kwargs):
        """Run require() against a stand-in guard that answers the request file."""
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            path, request = root / 'status.json', root / 'locks' / 'ossm-guard.request'
            if before is not None:
                path.write_text(json.dumps(before))
            seen = {}

            def guard():
                for _ in range(200):
                    if request.exists():
                        try:
                            seen.update(json.loads(request.read_text()))
                        except ValueError:
                            time.sleep(0.01)
                            continue
                        if answer is not None:
                            status = dict(after, recovery=dict(answer, id=seen['id']))
                            path.write_text(json.dumps(status))
                        return
                    time.sleep(0.01)
            thread = threading.Thread(target=guard); thread.start()
            quiet = MagicMock()
            watch = GuardWatch(path, root, request, 'researchanddesire/OSSM run 7-1')
            error = None
            try:
                watch.require(quiet=quiet, **kwargs)
            except RuntimeError as raised:
                error = str(raised)
            thread.join(); watch.close()
            record = json.loads((root / 'rs485-guard.json').read_text())
            return error, record, seen, quiet, request.exists()

    def test_latched_guard_is_recovered_before_the_run(self):
        latched = guard_status(activeFault='Hard-stop contact exceeds 200 ms', rail=dict(critical=True),
                               mode='strict', handback=None, safe=False)
        error, record, seen, quiet, left = self.recover(
            latched, dict(outcome='cleared', clearedFault='Hard-stop contact exceeds 200 ms'), guard_status())
        self.assertIsNone(error)
        quiet.assert_called_once()
        self.assertEqual(seen['by'], 'researchanddesire/OSSM run 7-1')
        self.assertEqual(record['recovery']['outcome'], 'cleared')
        self.assertIn('recovered automatically', record['preflight'])
        self.assertIn('Hard-stop contact exceeds 200 ms', record['preflight'])
        self.assertFalse(left)

    def test_missing_guard_is_waited_for(self):
        error, record, _, _, _ = self.recover(None, dict(outcome='not-needed'), guard_status())
        self.assertIsNone(error)
        self.assertEqual(record['recovery']['outcome'], 'not-needed')

    def test_refused_recovery_fails_before_any_write(self):
        latched = guard_status(activeFault='Motor alarm 4', rail=dict(critical=True), mode='strict',
                               handback=None, safe=False)
        error, record, _, _, left = self.recover(
            latched, dict(outcome='failed', error='motor not idle and healthy'), latched, settle_s=0.2)
        self.assertIn('Motor alarm 4', error)
        self.assertIn('automatic recovery: failed, motor not idle and healthy', error)
        self.assertEqual(record['recovery']['outcome'], 'failed')
        self.assertFalse(left)

    def test_silent_guard_fails_before_any_write(self):
        error, record, _, _, left = self.recover(None, None, None, wait_s=0.3, settle_s=0.2)
        self.assertIn('not running', error)
        self.assertEqual(record['recovery']['outcome'], 'no answer from the guard')
        self.assertFalse(left)

    def test_healthy_guard_is_never_asked_to_recover(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            (root / 'status.json').write_text(json.dumps(guard_status()))
            quiet = MagicMock()
            watch = GuardWatch(root / 'status.json', root, root / 'ossm-guard.request', 'run')
            watch.require(quiet=quiet); watch.close()
            quiet.assert_not_called()
            self.assertFalse((root / 'ossm-guard.request').exists())
            self.assertIsNone(json.loads((root / 'rs485-guard.json').read_text())['recovery'])

    def test_status_file_contention_retries_and_still_rejects_stale_samples(self):
        old = (datetime.now(timezone.utc) - timedelta(seconds=30)).isoformat()
        snapshot = json.dumps(guard_status(time=old))
        with patch.object(Path, 'read_text', side_effect=[PermissionError(), snapshot]) as reader:
            status = read_status('status.json')
        self.assertEqual(reader.call_count, 2)
        self.assertIn('guard is not running', unsafe_reason(status))

    def test_persistent_status_file_denial_fails(self):
        with patch.object(Path, 'read_text', side_effect=PermissionError()) as reader:
            with self.assertRaises(PermissionError):
                read_status('status.json')
        self.assertEqual(reader.call_count, 3)

    def test_fault_during_the_run_fails_it(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'status.json'
            path.write_text(json.dumps(guard_status()))
            watch = GuardWatch(path, root)
            watch.require()
            path.write_text(json.dumps(guard_status(faultCount=1, activeFault='Hard-stop contact exceeds 200 ms')))
            for _ in range(30):
                if watch.failure:
                    break
                watch._stop.wait(0.1)
            with self.assertRaises(RuntimeError):
                watch.check()
            watch.close()


class VideoTests(unittest.TestCase):
    def test_presets_are_validated(self):
        self.assertIn('hflip,vflip', view_filter(dict(crop=[0.1, 0.1, 0.5, 0.5], rotate=180), clock=False))
        for view in (dict(crop=[0.8, 0, 0.5, 1]), dict(crop=[0, 0, 1, 1], rotate=45), dict(crop=[0, 0, 0, 1])):
            with self.subTest(view=view), self.assertRaises(ValueError):
                view_filter(view)

    def test_missing_camera_configuration_records_why_without_raising(self):
        with tempfile.TemporaryDirectory() as root:
            recorder = Recorder(Path(root) / 'cameras.json', 'dtt', ['v1', 'v2'], root)
            recorder.start()
            recorder.event('v1', 'flashing')
            recorder.stop()
            manifest = json.loads((Path(root) / 'video' / 'manifest.json').read_text())
            self.assertIn('camera configuration unavailable', manifest['error'])
            self.assertEqual(manifest['views'], [])

    def test_camera_that_cannot_start_is_reported(self):
        with tempfile.TemporaryDirectory() as root:
            ffmpeg = Path(root) / 'ffmpeg.exe'
            ffmpeg.write_bytes(b'')
            config = dict(schema=1, ffmpeg=str(ffmpeg), cameras={'1': dict(device='cam', serial='S')},
                          products=dict(dtt=dict(camera='1', views=dict(v1=dict(crop=[0, 0, 1, 1])))))
            (Path(root) / 'cameras.json').write_text(json.dumps(config))
            recorder = Recorder(Path(root) / 'cameras.json', 'dtt', ['v1'], root)
            with patch('video.camera_service_up', return_value=False), \
                 patch('video.subprocess.Popen', side_effect=OSError('busy')):
                recorder.start()
            recorder.stop()
            manifest = json.loads((Path(root) / 'video' / 'manifest.json').read_text())
            self.assertIn('recording did not start', manifest['error'])


if __name__ == '__main__':
    unittest.main()
