"""Supplemental manual evidence cannot replace or relabel Test Rig results."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

PUBLISHER = Path(__file__).resolve().parents[2]/'.github'/'scripts'
sys.path.insert(0, str(PUBLISHER))
import hil_evidence as publisher  # noqa: E402
import hil_supplemental as supplemental  # noqa: E402

HEAD = 'a'*40
REPOSITORY = 'researchanddesire/ossm'


class SupplementalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.evidence = Path(self.temp.name)
        self.root = self.evidence/'supplemental'
        self.root.mkdir()
        self.video = self.root/'manual.mp4'
        # Synthetic parser input only. No video decoder, camera or device runs.
        self.video.write_bytes(b'\0\0\0\x18ftypisom'+b'offline fixture')
        self.controller = {'expectedBuild': HEAD, 'identity': {'model': 'OSSM', 'build': HEAD},
                           'completed': True, 'home': {'contacts': 2, 'boundsValid': True,
                                                       'positionSource': 'rs485'}}
        self.controller_path = self.root/'controller.json'
        self.manifest = {'schema': 1, 'kind': supplemental.KIND,
                         'repository': REPOSITORY, 'pr': 19, 'headSha': HEAD,
                         'guardMode': 'esp32', 'pcMonitor': 'paused',
                         'isRequiredTestRigValidation': False, 'requiredCiChecksChanged': False,
                         'video': {'file': self.video.name},
                         'controller': {'file': self.controller_path.name}}
        self.write_controller()
        self.manifest['video']['sha256'] = supplemental.sha256(self.video)
        self.write_manifest()

    def write_controller(self):
        self.controller_path.write_text(json.dumps(self.controller), encoding='utf8')
        self.manifest['controller']['sha256'] = supplemental.sha256(self.controller_path)

    def write_manifest(self):
        (self.root/'manifest.json').write_text(json.dumps(self.manifest), encoding='utf8')

    def load(self):
        return supplemental.load_supplemental(self.evidence, REPOSITORY, 19, HEAD)

    def test_valid_manual_evidence_has_separate_label_and_artifact_json(self):
        value = self.load()
        body = '\n'.join(supplemental.manual_section(value, None, 'https://example.test/run'))
        self.assertIn('### Supplemental manual ESP32 guard validation', body)
        self.assertIn('does not satisfy or change the required Test Rig checks', body)
        self.assertIn('JSON is not uploaded to the media-only storage bucket', body)
        self.assertNotIn('Hardware validation passed', body)

    def test_repository_pr_and_head_must_match(self):
        for key, value in (('repository', 'someone/ossm'), ('pr', 20), ('headSha', 'b'*40)):
            original = self.manifest[key]
            with self.subTest(key=key):
                self.manifest[key] = value
                self.write_manifest()
                with self.assertRaises(ValueError):
                    self.load()
            self.manifest[key] = original

    def test_runtime_and_expected_build_must_match_even_when_rehashed(self):
        for key in ('expectedBuild', 'identity'):
            old = self.controller[key]
            self.controller[key] = 'b'*40 if key == 'expectedBuild' else {'model': 'OSSM', 'build': 'b'*40}
            self.write_controller()
            self.write_manifest()
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.load()
            self.controller[key] = old

    def test_media_and_controller_hashes_must_match(self):
        for key in ('video', 'controller'):
            old = self.manifest[key]['sha256']
            self.manifest[key]['sha256'] = '0'*64
            self.write_manifest()
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.load()
            self.manifest[key]['sha256'] = old

    def test_paths_cannot_escape_or_use_absolute_windows_names(self):
        for name in ('../outside.mp4', '/outside.mp4', 'C:/outside.mp4', '..\\outside.mp4'):
            self.manifest['video']['file'] = name
            self.write_manifest()
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.load()

    def test_resolved_link_asset_cannot_escape(self):
        outside = self.evidence/'outside.mp4'
        outside.write_bytes(self.video.read_bytes())
        outside = outside.resolve()
        original = Path.resolve

        def resolve(path, *args, **kwargs):
            return outside if path.name == self.video.name else original(path, *args, **kwargs)

        # Covers symlink/junction resolution without requiring Windows link privileges.
        with patch.object(Path, 'resolve', resolve), self.assertRaises(ValueError):
            self.load()

    def test_manual_schema_cannot_claim_required_ci_validation(self):
        self.manifest['isRequiredTestRigValidation'] = True
        self.write_manifest()
        with self.assertRaises(ValueError):
            self.load()

    def test_absent_manual_manifest_is_optional(self):
        (self.root/'manifest.json').unlink()
        self.assertIsNone(self.load())

    def test_current_pr_head_requires_matching_event_and_live_head(self):
        event = {'number': 19, 'pull_request': {
            'base': {'repo': {'full_name': REPOSITORY}},
            'head': {'repo': {'full_name': REPOSITORY}, 'sha': HEAD}}}
        path = self.evidence/'event.json'
        path.write_text(json.dumps(event))
        with patch.dict(os.environ, GITHUB_EVENT_PATH=str(path)), patch.object(
                supplemental.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, HEAD+'\n')):
            self.assertEqual(supplemental.current_pr_head(REPOSITORY, 19), HEAD)
        with patch.dict(os.environ, GITHUB_EVENT_PATH=str(path)), patch.object(
                supplemental.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'b'*40+'\n')):
            with self.assertRaises(ValueError):
                supplemental.current_pr_head(REPOSITORY, 19)

    def test_failed_test_rig_result_stays_failed_with_successful_manual_evidence(self):
        results = self.evidence/'results'
        results.mkdir()
        (results/'results.json').write_text(json.dumps({'devices': [
            {'product': 'ossm', 'variant': '16mb', 'status': 'failed'}]}))
        posted = []

        def gh(command, **kwargs):
            if '--input' in command:
                posted.append(json.loads(kwargs['input'])['body'])
            return subprocess.CompletedProcess(command, 0, '')

        with patch.dict(os.environ, {'GITHUB_REPOSITORY': REPOSITORY, 'GITHUB_RUN_ID': '1',
                                     'GITHUB_RUN_ATTEMPT': '1', 'GITHUB_SHA': 'c'*40,
                                     'SUPABASE_ACCESS_TOKEN': ''}), \
                patch.object(sys, 'argv', ['hil_evidence.py', '--evidence', str(self.evidence),
                                          '--pr', '19', '--hardware-result', 'failure']), \
                patch.object(publisher, 'current_pr_head', return_value=HEAD), \
                patch.object(publisher.subprocess, 'run', side_effect=gh), redirect_stdout(io.StringIO()):
            self.assertEqual(publisher.main(), 0)
        self.assertEqual(len(posted), 1)
        self.assertIn('## ❌ Hardware validation failed', posted[0])
        self.assertIn('### Supplemental manual ESP32 guard validation', posted[0])
        self.assertNotIn('Hardware validation passed', posted[0])


if __name__ == '__main__':
    unittest.main()
