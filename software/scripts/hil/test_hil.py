import argparse
import hashlib
import io
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
from catalog import (PRODUCTS, REPOSITORIES, policy, regions, DTT_V1, OSSM_4MB, OSSM_16MB,
                     installed_layout_allowed, physical_flash_allowed)
from monitor import HAT_PREFIX, Monitor, EvidenceError, PREFIX
from run import RadarHold, find_port, load_fixture, probe, rig_lock, safe_line, test_device, main

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

    def test_only_the_16mb_ossm_may_also_serve_the_smaller_lane(self):
        self.assertTrue(physical_flash_allowed(policy('researchanddesire/OSSM', '16mb'), 16777216))
        self.assertTrue(physical_flash_allowed(policy('researchanddesire/OSSM', '16mb'), 16777216, True))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/OSSM', '16mb'), 16777216, 'yes'))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/OSSM', '16mb'), 4194304, True))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/dtt', 'v2'), 16777216, True))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/lkbx', 'r2'), 16777216, True))
        self.assertFalse(physical_flash_allowed(policy('researchanddesire/radr', 'r8'), 16777216, True))

    def test_dual_lane_ossm_enrolls_as_one_16mb_board(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory)/'fixture.json'
            device = dict(product='ossm', chip='esp32', flash_bytes=16777216, psram=None, enrolled=True,
                          device_id=MAC, usb_vid=4292, usb_pid=60000, usb_location='1-4.3.4.2',
                          allow_larger_flash=True)
            fixture.write_text(json.dumps(dict(schema=1, products=dict(ossm={'16mb': device}))))
            self.assertEqual(list(load_fixture(fixture, 'researchanddesire/OSSM')), ['16mb'])
            # The flag never relabels the board as a 4 MB fixture or adds a second capacity.
            fixture.write_text(json.dumps(dict(schema=1, products=dict(ossm={'4mb': device}))))
            with self.assertRaises(ValueError): load_fixture(fixture, 'researchanddesire/OSSM')
            fixture.write_text(json.dumps(dict(schema=1, products=dict(ossm={'4mb': device, '16mb': device}))))
            with self.assertRaises(ValueError): load_fixture(fixture, 'researchanddesire/OSSM')

    def test_dual_lane_ossm_may_restore_its_layout_after_a_v1_release(self):
        spec = policy('researchanddesire/OSSM', '16mb')
        self.assertTrue(installed_layout_allowed(spec, OSSM_16MB))
        self.assertTrue(installed_layout_allowed(spec, OSSM_4MB, True))
        self.assertFalse(installed_layout_allowed(spec, OSSM_4MB))
        self.assertFalse(installed_layout_allowed(spec, DTT_V1, True))
        self.assertFalse(installed_layout_allowed(policy('researchanddesire/OSSM', '4mb'), OSSM_16MB, True))
        self.assertFalse(installed_layout_allowed(policy('researchanddesire/dtt', 'v2'), DTT_V1, True))
        # Both reviewed OSSM layouts keep NVS in place, so the write's NVS check still holds.
        self.assertEqual([r for r in OSSM_4MB if r[0] == 'nvs'], [r for r in OSSM_16MB if r[0] == 'nvs'])

    def test_write_replaces_the_v1_layout_only_on_the_dual_lane_ossm(self):
        spec = policy('researchanddesire/OSSM', '16mb')
        table = b''.join(struct.pack('<HBBII16sI', 0x50aa, row[1], row[2], row[3], row[4], row[0].encode(), 0)
                         for row in OSSM_4MB)+b'\xff'*32
        def connected_board():
            connected = MagicMock()
            connected.run_stub.return_value = connected
            connected.CHIP_NAME = 'ESP32'
            connected.read_mac.return_value = bytes.fromhex(MAC)
            connected.flash_id.return_value = 24 << 16
            connected.read_flash.side_effect = lambda offset, size: table if offset == 0x8000 else b'nvs'
            return connected
        for allowed in (True, False):
            with self.subTest(allowed=allowed), patch('write.esptool.detect_chip', return_value=connected_board()), \
                 patch('write.esptool.main') as writer:
                if allowed:
                    write_verified('COM99', spec, MAC, Path('bundle'), 16777216, True)
                    offsets = writer.call_args.args[0]
                    self.assertIn('0x8000', offsets)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'installed partitions changed'):
                        write_verified('COM99', spec, MAC, Path('bundle'), 16777216, False)
                    writer.assert_not_called()

    def test_probe_accepts_the_v1_layout_only_on_the_dual_lane_ossm(self):
        spec = policy('researchanddesire/OSSM', '16mb')
        table = b''.join(struct.pack('<HBBII16sI', 0x50aa, row[1], row[2], row[3], row[4], row[0].encode(), 0)
                         for row in OSSM_4MB)+b'\xff'*32
        def esptool(port, spec, args, log, timeout=90):
            if args[0] == 'read_flash':
                Path(args[3]).write_bytes(table)
                return ''
            return 'MAC: aa:bb:cc:dd:ee:ff\nDetected flash size: 16MB\n'
        device = dict(device_id=MAC, flash_bytes=16777216)
        with patch('run.esptool', side_effect=esptool):
            probe('COM99', spec, dict(device, allow_larger_flash=True), None)
            with self.assertRaisesRegex(RuntimeError, 'installed partition layout differs'):
                probe('COM99', spec, device, None)

    def test_production_and_main_track_health_lines_are_kept(self):
        boot = {'event': 'boot', 'boot_id': 7, 'uptime_ms': 812, 'build_sha': 'a'*40, 'track': 'main',
                'variant': '4mb', 'device_id': MAC, 'image_sha256': 'b'*64, 'reset_reason': 1, 'hat_mode': 'off'}
        heartbeat = {'event': 'heartbeat', 'boot_id': 7, 'uptime_ms': 5000, 'main_menu': True,
                     'menu_age_ms': 12, 'hat': False, 'homed': True}
        for prefix in ('LKBX_HEALTH ', 'OSSM_HEALTH ', 'DTT_HEALTH ', 'RADR_HEALTH '):
            for record in (boot, heartbeat, {'event': 'wifi_connected', 'boot_id': 7, 'uptime_ms': 9, 'ip': '10.0.0.4'},
                           {'event': 'cloud_ok', 'boot_id': 7, 'uptime_ms': 9},
                           {'event': 'heartbeat', 'boot_id': 7, 'uptime_ms': 9, 'ram_free': 4, 'ram_largest': 2}):
                line = prefix+json.dumps(record)
                self.assertEqual(json.loads(safe_line('I (12) x: '+line).split(prefix, 1)[1]), record)
        broker = {'event': 'mqtt_connected', 'boot_id': 7, 'uptime_ms': 9,
                  'broker': 'mqtts://te9acc16.ala.us-east-1.emqxsl.com'}
        self.assertIn('emqxsl.com', safe_line('OSSM_HEALTH '+json.dumps(broker)))
        for unsafe in ({**broker, 'broker': 'mqtts://user:secret-value@host'}, {**boot, 'password': 'secret-value'},
                       {**boot, 'track': 'secret-value'}, {**heartbeat, 'hat': 'secret-value'}):
            self.assertNotIn('secret-value', safe_line('OSSM_HEALTH '+json.dumps(unsafe)))
        self.assertNotIn('secret-value', safe_line('OSSM_HEALTH {"event":"boot","note":"secret-value"'))
        staging = {'event': 'boot', 'boot_id': 1, 'uptime_ms': 2, 'track': 'main', 'product': 'ossm'}
        self.assertEqual(json.loads(safe_line(PREFIX+json.dumps(staging))[len(PREFIX):]), staging)

    def test_hat_switch_reply_and_mode_are_kept(self):
        reply = {'mode': 'off', 'applies': 'next_boot'}
        self.assertEqual(json.loads(safe_line(HAT_PREFIX+json.dumps(reply))[len(HAT_PREFIX):]), reply)
        self.assertNotIn('secret-value', safe_line(HAT_PREFIX+json.dumps({**reply, 'x': 'secret-value'})))
        self.assertNotIn('secret-value', safe_line(HAT_PREFIX+json.dumps({'mode': 'secret-value', 'applies': 'next_boot'})))
        status = dict(detected=True, fresh=True, passive=True, guardReady=True, guardArmed=False,
                      guardLatched=False, outputDisabled=True, guardFault=0)
        kept = json.loads(safe_line(HAT_PREFIX+json.dumps({**status, 'mode': 'auto', 'ageMs': 3}))[len(HAT_PREFIX):])
        self.assertEqual(kept, {**status, 'mode': 'auto'})

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

    def test_usb_location_matches_with_or_without_the_interface_suffix(self):
        def port(device, location, serial_number=None, vid=4292, pid=60000):
            return MagicMock(device=device, location=location, serial_number=serial_number, vid=vid, pid=pid)
        ports = [port('COM6', '1-4.3.4.2:x.0'), port('COM5', '1-4.4:x.0'), port('COM7', '1-4.3.3:x.0'),
                 port('COM4', '1-4.2', '0001'), port('COM10', None, 'D8:3B', 12346, 4097)]
        fixture = lambda **identity: dict(usb_vid=4292, usb_pid=60000, **identity)
        with patch('serial.tools.list_ports.comports', return_value=ports):
            self.assertEqual(find_port(fixture(usb_location='1-4.3.4.2')), 'COM6')
            self.assertEqual(find_port(fixture(usb_location='1-4.3.3:x.0')), 'COM7')
            self.assertEqual(find_port(fixture(usb_serial='0001')), 'COM4')
            # A deeper path is a different socket, and a port without a location never matches.
            with self.assertRaisesRegex(RuntimeError, 'exactly one USB identity'):
                find_port(fixture(usb_location='1-4.3.4'))
            with self.assertRaisesRegex(RuntimeError, 'exactly one USB identity'):
                find_port(dict(usb_vid=12346, usb_pid=4097, usb_location=None))

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



RADAR_DEVICE = dict(product='radr', enrolled=True, chip='esp32s3', flash_bytes=16777216, psram='r8',
                    device_id='A4CB8FC2DCB8', usb_vid=4292, usb_pid=60000, usb_serial='0001')
OSSM_DEVICE = dict(product='ossm', enrolled=True, chip='esp32', flash_bytes=16777216, psram=None,
                   device_id='4CC3821011DC', usb_vid=4292, usb_pid=60000, usb_location='1-4.3.4.2')


class RadarIsolationTests(unittest.TestCase):
    """OSSM PR runs keep the rig's Radar in its ROM loader, then park the OSSM before releasing it."""
    def setUp(self):
        self.calls, self.failing, self.macs = [], set(), {}

    def esptool(self, port, spec, args, log, timeout=90, after='no_reset'):
        self.calls.append((port, args[0], after))
        if port in self.failing:
            raise RuntimeError('esptool failed: '+args[0])
        mac = self.macs.get(port, {'COM4': RADAR_DEVICE, 'COM6': OSSM_DEVICE}[port]['device_id'])
        return 'MAC: ' + ':'.join(mac[i:i+2] for i in range(0, 12, 2)).lower() + '\n'

    def port(self, device):
        return 'COM4' if device['product'] == 'radr' else 'COM6'

    def hold(self, directory, events):
        return RadarHold(RADAR_DEVICE, {'16mb': OSSM_DEVICE}, 'researchanddesire/OSSM', Path(directory),
                         lambda variant, name: events.append((variant, name)))

    def test_the_ossm_is_parked_before_the_radar_is_released(self):
        events = []
        with tempfile.TemporaryDirectory() as directory, patch('run.esptool', side_effect=self.esptool), \
             patch('run.find_port', side_effect=self.port):
            hold = self.hold(directory, events)
            hold.hold()
            hold.release()
            hold.release()  # idempotent
            self.assertTrue(Path(directory, 'radar-hold.log').exists())
        self.assertEqual(self.calls, [('COM4', 'read_mac', 'no_reset'), ('COM6', 'read_mac', 'no_reset'),
                                      ('COM4', 'read_mac', 'hard_reset')])
        self.assertEqual(hold.record, dict(enrolled=True, held=True, ossm_parked=True, released=True, errors=[]))
        self.assertEqual([variant for variant, _ in events], ['radr', '16mb', 'radr'])

    def test_the_radar_stays_held_when_the_ossm_cannot_be_parked(self):
        events = []
        self.failing.add('COM6')
        with tempfile.TemporaryDirectory() as directory, patch('run.esptool', side_effect=self.esptool), \
             patch('run.find_port', side_effect=self.port):
            hold = self.hold(directory, events)
            hold.hold()
            hold.release()
        self.assertNotIn(('COM4', 'read_mac', 'hard_reset'), self.calls)
        self.assertFalse(hold.record['released'])
        self.assertIn('the Radar stays held', hold.record['errors'][0])
        self.assertEqual(events[-1], ('radr', 'left in its ROM loader: the OSSM could not be parked'))

    def test_a_wrong_board_on_either_port_is_not_accepted(self):
        self.macs['COM4'] = 'AABBCCDDEEFF'
        with tempfile.TemporaryDirectory() as directory, patch('run.esptool', side_effect=self.esptool), \
             patch('run.find_port', side_effect=self.port):
            with self.assertRaisesRegex(RuntimeError, 'factory MAC'):
                self.hold(directory, []).hold()
            untouched = self.hold(directory, [])
            untouched.release()  # never held: nothing to park or release
        self.assertEqual(self.calls, [('COM4', 'read_mac', 'no_reset')])

    def run_main(self, repository='researchanddesire/OSSM', radar_enrolled=True):
        product = REPOSITORIES[repository]
        devices = {'16mb': OSSM_DEVICE} if product == 'ossm' else {'v1': {'port': 'COM7'}, 'v2': {'port': 'COM8'}}
        self.timeline, self.locks, self.fixtures = [], [], []
        def fixtures(path, repo):
            self.fixtures.append(repo)
            if repo == 'researchanddesire/radr':
                if not radar_enrolled:
                    raise ValueError('fixture does not cover every required variant')
                return {'r8': RADAR_DEVICE}
            return devices
        def lock(path, **kwargs):
            self.locks.append(Path(path).name)
            return MagicMock()
        def esptool(port, spec, args, log, timeout=90, after='no_reset'):
            self.timeline.append(f'esptool {port} {after}')
            return self.esptool(port, spec, args, log, timeout, after)
        class Recorder:
            def __init__(rec, *args): pass
            def start(rec): pass
            def stop(rec): pass
            def event(rec, variant, name): self.timeline.append(f'event {variant}: {name}')
        class Guard:
            def __init__(guard, *args): pass
            def require(guard, quiet=None): self.timeline.append('guard')
            def close(guard): pass
        def device(variant, *args):
            self.timeline.append('test_device')
            return dict(variant=variant, status='passed')
        manifest = dict(repository=repository, product=product)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            argv = ['run.py', '--repository', repository, '--build-sha', SHA, '--run-id', '42', '--run-attempt', '1',
                    '--bundle', str(root/'bundle'), '--fixture', str(root/'fixture.json'),
                    '--output', str(root/'output'), '--lock-directory', str(root/'locks')]
            with patch('sys.argv', argv), patch('run.validate', return_value=manifest), \
                 patch('run.load_fixture', side_effect=fixtures), patch('run.rig_lock', side_effect=lock), \
                 patch('run.find_port', side_effect=lambda d: d.get('port') or self.port(d)), \
                 patch('run.esptool', side_effect=esptool), patch('run.Recorder', Recorder), \
                 patch('run.GuardWatch', Guard), patch('run.test_device', side_effect=device):
                code = main()
            report = json.loads((root/'output'/'results.json').read_text())
        return code, report

    def test_an_ossm_pr_run_holds_the_radar_for_the_whole_run(self):
        code, report = self.run_main()
        self.assertEqual(code, 0)
        self.assertEqual(self.locks, ['ossm.lock', 'radr.lock'])
        line = self.timeline
        self.assertLess(line.index('esptool COM4 no_reset'), line.index('guard'))
        self.assertLess(line.index('guard'), line.index('test_device'))
        self.assertLess(line.index('test_device'), line.index('esptool COM6 no_reset'))
        self.assertLess(line.index('esptool COM6 no_reset'), line.index('esptool COM4 hard_reset'))
        self.assertEqual(report['radar'], dict(enrolled=True, held=True, ossm_parked=True, released=True, errors=[]))
        self.assertIn('event radr: held in its ROM loader so it cannot connect to the OSSM', line)

    def test_a_rig_without_a_radar_runs_the_ossm_and_says_so(self):
        out = io.StringIO()
        with patch('sys.stdout', out):
            code, report = self.run_main(radar_enrolled=False)
        self.assertEqual(code, 0)
        self.assertEqual(self.locks, ['ossm.lock'])
        self.assertFalse(any('COM4' in step for step in self.timeline))
        self.assertEqual(report['radar'], dict(enrolled=False))
        self.assertIn('radar: not enrolled on this rig', out.getvalue())

    def test_a_radar_that_cannot_be_held_stops_the_ossm_run_before_any_write(self):
        self.failing.add('COM4')
        code, report = self.run_main()
        self.assertEqual(code, 1)
        self.assertEqual(report['devices'][0]['variant'], 'preflight')
        self.assertIn('refusing to run the OSSM beside it', report['devices'][0]['error'])
        self.assertNotIn('guard', self.timeline)
        self.assertNotIn('test_device', self.timeline)
        self.assertFalse(report['radar']['held'])

    def test_other_products_never_touch_the_radar(self):
        code, report = self.run_main('researchanddesire/dtt')
        self.assertEqual(code, 0)
        self.assertEqual(self.fixtures, ['researchanddesire/dtt'])
        self.assertNotIn('radr.lock', self.locks)
        self.assertNotIn('radar', report)


if __name__ == '__main__':
    unittest.main()
