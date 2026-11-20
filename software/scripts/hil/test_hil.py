import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch, MagicMock
from write import write_verified

from bundle import validate, package, merge, partition_rows, validate_partitions
from catalog import PRODUCTS, REPOSITORIES, policy, regions, OSSM_4MB, physical_flash_allowed
from monitor import HAT_PREFIX, Monitor, EvidenceError, PREFIX
from run import load_fixture, rig_lock, safe_line, test_device, main

SHA = 'a'*40
MAC = 'AABBCCDDEEFF'


def image(spec):
    data = bytearray(256)
    data[0], data[23] = 0xe9, 1
    data[3] = (2 if spec['flash_bytes'] == 4*1024*1024 else 4) << 4
    struct.pack_into('<H', data, 12, 9 if spec['chip'] == 'esp32s3' else 0)
    return bytes(data)+hashlib.sha256(data).digest()


def make_bundle(root, repository):
    for variant in PRODUCTS[REPOSITORIES[repository]]['variants']:
        spec = policy(repository, variant)
        source = root/'build'/variant
        source.mkdir(parents=True)
        for filename in regions(spec):
            data = b'non-executable fixture'
            if filename in ('firmware.bin', 'bootloader.bin'):
                data = image(spec)
            elif filename == 'partitions.bin':
                data = b''.join(struct.pack('<HBBII16sI', 0x50aa, row[1], row[2], row[3], row[4], row[0].encode(), 0)
                                for row in spec['partitions'])+b'\xff'*32
            (source/filename).write_bytes(data)
        args = argparse.Namespace(repository=repository, variant=variant, build_sha=SHA,
                                  run_id='42', run_attempt='1', output=root/'bundle', build_dir=source)
        package(args)
    merge(args)
    return root/'bundle'


class BundleTests(unittest.TestCase):
    def test_ossm_legacy_layout_is_not_silently_migrated(self):
        data = b''.join(struct.pack('<HBBII16sI', 0x50aa, row[1], row[2], row[3], row[4], row[0].encode(), 0)
                        for row in OSSM_4MB)+b'\xff'*32
        validate_partitions(data, policy('researchanddesire/OSSM', '4mb'))
        with self.assertRaisesRegex(ValueError, 'partition layout differs'):
            validate_partitions(data, policy('researchanddesire/OSSM', '16mb'))

    def test_every_product_variant_packages_and_validates(self):
        for repo in REPOSITORIES:
            with self.subTest(repo=repo), tempfile.TemporaryDirectory() as directory:
                bundle = make_bundle(Path(directory), repo)
                result = validate(bundle, repo, SHA, '42', '1')
                self.assertEqual(set(result['variants']), set(PRODUCTS[REPOSITORIES[repo]]['variants']))

    def test_rejects_wrong_repository_sha_run_and_attempt(self):
        repo = 'researchanddesire/dtt'
        with tempfile.TemporaryDirectory() as directory:
            bundle = make_bundle(Path(directory), repo)
            for identity in [(repo, 'b'*40, '42', '1'), (repo, SHA, '43', '1'),
                             (repo, SHA, '42', '2'), ('researchanddesire/lkbx', SHA, '42', '1')]:
                with self.subTest(identity=identity), self.assertRaises(ValueError):
                    validate(bundle, *identity)

    def test_corruption_wrong_flash_chip_offset_or_missing_variant_fails(self):
        repo = 'researchanddesire/dtt'
        for mutation in ('corrupt', 'flash', 'chip', 'offset', 'missing', 'overflow', 'partition'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                bundle = make_bundle(Path(directory), repo)
                manifest = json.loads((bundle/'manifest.json').read_text())
                entry = manifest['variants']['v1']
                if mutation == 'missing':
                    del manifest['variants']['v2']
                elif mutation == 'offset':
                    entry['components']['firmware.bin']['offset'] = 0x9000
                else:
                    filename = 'partitions.bin' if mutation == 'partition' else 'firmware.bin'
                    data = bytearray((bundle/'v1'/filename).read_bytes())
                    if mutation == 'flash': data[3] = 0x40
                    elif mutation == 'chip': data[12] = 9
                    elif mutation == 'partition': data[8] ^= 1
                    elif mutation == 'overflow': data = bytearray(0x140001)
                    else: data[100] ^= 1
                    (bundle/'v1'/filename).write_bytes(data)
                    if mutation != 'corrupt':
                        entry['components'][filename].update(size=len(data), sha256=hashlib.sha256(data).hexdigest())
                (bundle/'manifest.json').write_text(json.dumps(manifest))
                with self.assertRaises(ValueError): validate(bundle, repo, SHA, '42', '1')


class MonitorTests(unittest.TestCase):
    def monitor(self, product='dtt'):
        expected = dict(schema=1, product=product, variant='v1' if product == 'dtt' else '4mb',
                        device_id=MAC, flash_bytes=4194304, build_sha=SHA, image_sha256='b'*64,
                        track='staging')
        monitor = Monitor(expected, 0)
        monitor.feed(PREFIX+json.dumps(dict(expected, event='boot', boot_id=1, uptime_ms=10)), .01)
        return monitor

    def heartbeat(self, monitor, now, **changes):
        data = dict(event='heartbeat', boot_id=1, uptime_ms=int(now*1000), ready=True,
                    app_age_ms=50, wifi=True, bench_wifi=True, ip='192.168.1.10',
                    internet=True, network_age_ms=1000, disconnects=0)
        data.update(changes)
        monitor.feed(PREFIX+json.dumps(data), now)

    def test_requires_full_host_and_device_observation(self):
        monitor = self.monitor()
        for now in range(5, 185, 5): self.heartbeat(monitor, now)
        self.assertFalse(monitor.passed)
        self.heartbeat(monitor, 185)
        self.assertTrue(monitor.passed)

    def test_host_time_cannot_replace_device_uptime(self):
        monitor = self.monitor()
        for now in range(5, 190, 5): self.heartbeat(monitor, now, uptime_ms=now*100)
        self.assertFalse(monitor.passed)

    def test_faults_and_network_losses_cannot_restart_window(self):
        failures = [dict(boot_id=2), dict(uptime_ms=1), dict(ready=False), dict(app_age_ms=20001),
                    dict(wifi=False), dict(bench_wifi=False), dict(ip='0.0.0.0'),
                    dict(internet=False), dict(network_age_ms=60001), dict(disconnects=1)]
        for change in failures:
            with self.subTest(change=change):
                monitor = self.monitor(); self.heartbeat(monitor, 5)
                with self.assertRaises(EvidenceError): self.heartbeat(monitor, 10, **change)

    def test_missing_malformed_boot_or_fault_text_fails(self):
        for line in ('RAD_HEALTH {}', 'RAD_HEALTH [1]', 'RAD_HEALTH {', 'Guru Meditation', 'watchdog', 'rst:0x1'):
            with self.subTest(line=line), self.assertRaises(EvidenceError):
                self.monitor().feed(line, 10)
        monitor = self.monitor(); self.heartbeat(monitor, 5)
        with self.assertRaises(EvidenceError): monitor.tick(26)
        with self.assertRaises(EvidenceError): self.monitor().tick(181)

    def test_wrong_running_identity_and_second_boot_fail(self):
        monitor = self.monitor()
        record = dict(monitor.expected, event='boot', boot_id=2, uptime_ms=10)
        with self.assertRaises(EvidenceError): monitor.feed(PREFIX+json.dumps(record), 11)
        fresh = Monitor(monitor.expected, 0)
        record['device_id'] = '112233445566'
        with self.assertRaises(EvidenceError): fresh.feed(PREFIX+json.dumps(record), 1)

    def test_ossm_reports_latched_stop_path_before_startup_deadline(self):
        monitor = self.monitor('ossm')
        status = dict(detected=True, fresh=True, passive=True, guardReady=False,
                      guardArmed=False, guardLatched=True, guardFault=16, outputDisabled=False)
        line = HAT_PREFIX+json.dumps(dict(status, other='secret-value'))
        self.monitor().feed(line, 5)
        with self.assertRaisesRegex(EvidenceError, r'StopPathUnverified.*shared RS-485 bus'):
            monitor.feed(line, 5)
        self.assertEqual(monitor.result(5)['ossm_hat'], status)
        self.assertFalse(monitor.passed)
        self.assertEqual(json.loads(safe_line(line)[len(HAT_PREFIX):]), status)

    def test_unlatched_hat_does_not_override_normal_readiness(self):
        monitor = self.monitor('ossm')
        status = dict(detected=True, fresh=True, passive=True, guardReady=False,
                      guardArmed=False, guardLatched=False, guardFault=0, outputDisabled=True)
        monitor.feed(HAT_PREFIX+json.dumps(status), 2)
        self.assertIsNone(monitor.soak_started)
        self.heartbeat(monitor, 5)
        self.assertEqual(monitor.soak_started, 5)
        self.assertFalse(monitor.passed)


class HostTests(unittest.TestCase):
    def test_verified_connection_is_reused_without_loading_a_second_stub(self):
        spec = policy('researchanddesire/dtt', 'v1')
        connected = MagicMock()
        connected.run_stub.return_value = connected
        connected.CHIP_NAME = 'ESP32'
        connected.read_mac.return_value = bytes.fromhex(MAC)
        connected.flash_id.return_value = 22 << 16
        connected.read_flash.return_value = b''.join(
            struct.pack('<HBBII16sI', 0x50aa, row[1], row[2], row[3], row[4], row[0].encode(), 0)
            for row in spec['partitions'])+b'\xff'*32
        with patch('write.esptool.detect_chip', return_value=connected), patch('write.esptool.main') as writer:
            write_verified('COM99', spec, MAC, Path('bundle'))
            self.assertIn('--no-stub', writer.call_args.args[0])
            self.assertIn('--compress', writer.call_args.args[0])
            self.assertEqual(writer.call_args.args[0][writer.call_args.args[0].index('--after')+1], 'no_reset_stub')
            self.assertIs(writer.call_args.kwargs['esp'], connected)
            connected.run_stub.assert_called_once()
            self.assertEqual(connected.WRITE_FLASH_ATTEMPTS, 1)

    def test_nvs_change_after_write_fails_validation(self):
        spec = policy('researchanddesire/dtt', 'v1')
        connected = MagicMock()
        connected.run_stub.return_value = connected
        connected.read_flash.side_effect = [b'preserved identity', b'changed identity']
        with patch('write.esptool.detect_chip', return_value=connected), \
             patch('write.verify_connected'), patch('write.esptool.main'):
            with self.assertRaisesRegex(RuntimeError, 'NVS changed'):
                write_verified('COM99', spec, MAC, Path('bundle'))
        connected._port.close.assert_called_once()

    def test_larger_trainer_requires_explicit_enrollment_exception(self):
        spec = policy('researchanddesire/dtt', 'v1')
        self.assertFalse(physical_flash_allowed(spec, 16777216))
        self.assertTrue(physical_flash_allowed(spec, 16777216, True))
        self.assertFalse(physical_flash_allowed(spec, 8388608, True))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/OSSM', '4mb'), 16777216, True))
    def test_swapped_device_cannot_reach_write_flash(self):
        spec = policy('researchanddesire/dtt', 'v1')
        connected = MagicMock()
        connected.run_stub.return_value = connected
        connected.CHIP_NAME = 'ESP32'
        connected.read_mac.return_value = bytes.fromhex('112233445566')
        with patch('write.esptool.detect_chip', return_value=connected), patch('write.esptool.main') as writer:
            with self.assertRaisesRegex(RuntimeError, 'factory MAC changed'):
                write_verified('COM99', spec, MAC, Path('bundle'))
            writer.assert_not_called()
            connected._port.close.assert_called_once()

    def test_sensitive_unstructured_logs_are_redacted(self):
        self.assertEqual(safe_line('RAD_STATE pairing'), 'RAD_STATE pairing')
        self.assertNotIn('secret-value', safe_line('RAD_STATE secret-value'))
        self.assertNotIn('secret-value', safe_line('password=secret-value'))
        self.assertNotIn('secret-value', safe_line('RAD_HEALTH {"password":"secret-value"}'))
        for key in ('event', 'build_sha', 'uptime_ms', 'ip', 'ready'):
            self.assertNotIn('secret-value', safe_line(PREFIX+json.dumps({key: 'secret-value'})))

    def test_missing_or_wrong_capacity_fixture_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory)/'fixture.json'
            fixture.write_text('{"products": {"dtt": {}}}')
            with self.assertRaises(ValueError): load_fixture(fixture, 'researchanddesire/dtt')

    def test_wrong_capacity_or_duplicate_identity_fails_enrollment(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory)/'fixture.json'
            devices = {}
            for n, variant in enumerate(('v1', 'v2')):
                spec = policy('researchanddesire/dtt', variant)
                devices[variant] = dict(product='dtt', chip='esp32', flash_bytes=spec['flash_bytes'],
                                        psram=None, enrolled=True, device_id=f'AABBCCDDEE{n:02X}',
                                        usb_vid=0x10c4, usb_pid=0xea60, usb_location=f'usb-{n}')
            fixture.write_text(json.dumps(dict(schema=1, products=dict(dtt=devices))))
            self.assertEqual(len(load_fixture(fixture, 'researchanddesire/dtt')), 2)
            devices['v1']['flash_bytes'] = 16777216
            fixture.write_text(json.dumps(dict(schema=1, products=dict(dtt=devices))))
            with self.assertRaises(ValueError): load_fixture(fixture, 'researchanddesire/dtt')
            devices['v1']['flash_bytes'] = 4194304
            devices['v2']['device_id'] = devices['v1']['device_id']
            fixture.write_text(json.dumps(dict(schema=1, products=dict(dtt=devices))))
            with self.assertRaises(ValueError): load_fixture(fixture, 'researchanddesire/dtt')

    def test_flash_interruption_or_serial_failure_is_not_retried(self):
        repo = 'researchanddesire/dtt'
        for stage in ('probe', 'flash', 'observe'):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                bundle = make_bundle(root, repo)
                manifest = validate(bundle, repo, SHA, '42', '1')
                with patch('run.find_port', return_value='COM99'), patch('run.probe') as probe, \
                     patch('run.flash') as flash, patch('run.observe') as observe:
                    {'probe': probe, 'flash': flash, 'observe': observe}[stage].side_effect = OSError('interrupted')
                    result = test_device('v1', dict(device_id=MAC, flash_bytes=4194304), manifest, bundle, root)
                    self.assertEqual(result['status'], 'failed')
                    self.assertEqual(result['error'], 'interrupted')
                    self.assertLessEqual(flash.call_count, 1)
                    if stage != 'observe': observe.assert_not_called()
                    self.assertTrue((root/'v1-result.json').exists())

    def test_missing_device_prevents_all_writes_and_produces_junit(self):
        repo = 'researchanddesire/dtt'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = make_bundle(root, repo)
            args = ['run.py', '--repository', repo, '--build-sha', SHA, '--run-id', '42', '--run-attempt', '1',
                    '--bundle', str(bundle), '--fixture', str(root/'fixture.json'),
                    '--output', str(root/'output'), '--lock-directory', str(root/'locks')]
            with patch('sys.argv', args), patch('run.load_fixture', return_value={'v1': {}, 'v2': {}}), \
                 patch('run.find_port', side_effect=['COM99', RuntimeError('missing device')]), patch('run.flash') as flash:
                self.assertEqual(main(), 1)
                flash.assert_not_called()
                self.assertIn('failures="1"', (root/'output/junit.xml').read_text())

    def test_os_lock_blocks_another_repository_process(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'shared.lock'
            with rig_lock(path):
                code = 'from pathlib import Path; from run import rig_lock\nwith rig_lock(Path(__import__("sys").argv[1])): pass'
                result = subprocess.run([sys.executable, '-c', code, str(path)], cwd=Path(__file__).parent,
                                        capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('another process owns', result.stderr)
            with rig_lock(path): pass

    def test_lock_names_its_holder_and_forgets_it_on_release(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ossm.lock'
            holder = Path(directory)/'ossm.lock.holder'
            with rig_lock(path, owner='bench session'):
                note = json.loads(holder.read_text())
                self.assertEqual((note['pid'], note['owner'], note['priority']), (os.getpid(), 'bench session', False))
            self.assertFalse(holder.exists())

    def test_pull_request_run_takes_the_fixture_from_a_manual_holder(self):
        # The child stands in for a bench session; this test plays the rig watchdog.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ossm.lock'
            code = ('import sys, time; from pathlib import Path; from run import rig_lock\n'
                    'with rig_lock(Path(sys.argv[1]), owner="bench session"):\n'
                    '    print("held", flush=True); time.sleep(60)')
            child = subprocess.Popen([sys.executable, '-c', code, str(path)], cwd=Path(__file__).parent,
                                     stdout=subprocess.PIPE, text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(), 'held')
                preempt = Path(directory)/'ossm.lock.preempt'
                seen = {}

                def watchdog():
                    for _ in range(100):
                        if preempt.exists():
                            seen['request'] = json.loads(preempt.read_text())
                            seen['holder'] = json.loads((Path(directory)/'ossm.lock.holder').read_text())
                            child.kill()
                            return
                        time.sleep(0.05)
                thread = threading.Thread(target=watchdog); thread.start()
                with rig_lock(path, priority=True, owner='researchanddesire/OSSM run 7-1', wait_s=20):
                    thread.join()
                    self.assertEqual(seen['request']['pid'], os.getpid())
                    self.assertEqual(seen['holder']['pid'], child.pid)
                    self.assertFalse(seen['holder']['priority'])
                    self.assertFalse(preempt.exists())
                    now = json.loads((Path(directory)/'ossm.lock.holder').read_text())
                    self.assertTrue(now['priority'])
            finally:
                child.kill(); child.wait(); child.stdout.close()

    def test_pull_request_run_gives_up_when_the_fixture_is_never_freed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ossm.lock'
            with rig_lock(path):
                code = ('import sys; from pathlib import Path; from run import rig_lock\n'
                        'with rig_lock(Path(sys.argv[1]), priority=True, wait_s=1.2): pass')
                result = subprocess.run([sys.executable, '-c', code, str(path)], cwd=Path(__file__).parent,
                                        capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('another process owns', result.stderr)
            self.assertFalse((Path(directory)/'ossm.lock.preempt').exists())


if __name__ == '__main__':
    unittest.main()
