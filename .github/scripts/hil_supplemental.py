"""Validate optional manual ESP32-guard evidence without changing CI results."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

KIND = 'supplemental-manual-esp32-guard'
SHA = re.compile(r'^[0-9a-f]{40}$')
HASH = re.compile(r'^[0-9a-f]{64}$')
NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')


def current_pr_head(repository, pr):
    """Bind supplemental evidence to this event and the PR's current head."""
    try:
        event = json.loads(Path(os.environ['GITHUB_EVENT_PATH']).read_text(encoding='utf-8'))
        pull = event['pull_request']
        if (event['number'] != int(pr) or
                pull['base']['repo']['full_name'].lower() != repository.lower() or
                pull['head']['repo']['full_name'].lower() != repository.lower()):
            raise ValueError()
        expected = pull['head']['sha']
        if not isinstance(expected, str) or not SHA.fullmatch(expected):
            raise ValueError()
        live = subprocess.run(['gh', 'api', f'repos/{repository}/pulls/{pr}', '--jq', '.head.sha'],
                              capture_output=True, text=True, check=True).stdout.strip()
        if live != expected:
            raise ValueError()
        return live
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        raise ValueError('current same-repository PR head could not be verified') from None


def read_json(path, limit):
    try:
        if not path.is_file() or not 0 < path.stat().st_size <= limit:
            raise ValueError()
        value = json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(value, dict):
            raise ValueError()
        return value
    except (OSError, ValueError):
        raise ValueError('supplemental JSON is missing, oversized, or malformed') from None


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def asset(root, descriptor, extension, limit):
    if not isinstance(descriptor, dict):
        raise ValueError('supplemental asset descriptor is missing')
    name, digest = descriptor.get('file'), descriptor.get('sha256')
    if (not isinstance(name, str) or not NAME.fullmatch(name) or
            Path(name).suffix.lower() != extension or
            not isinstance(digest, str) or not HASH.fullmatch(digest)):
        raise ValueError('supplemental asset name or SHA-256 is invalid')
    try:
        path = (root/name).resolve(strict=True)
        if path.parent != root or not path.is_file() or not 0 < path.stat().st_size <= limit:
            raise ValueError()
    except (OSError, ValueError):
        raise ValueError('supplemental asset escapes its directory, is absent, or exceeds its size limit') from None
    if sha256(path) != digest:
        raise ValueError('supplemental asset SHA-256 does not match')
    return path


def load_supplemental(evidence, repository, pr, head_sha):
    candidate = evidence/'supplemental'
    if not (candidate/'manifest.json').exists():
        return None
    root = candidate.resolve()
    if root.parent != evidence.resolve():
        raise ValueError('supplemental directory escapes the evidence artifact')
    manifest_path = root/'manifest.json'
    if manifest_path.resolve().parent != root:
        raise ValueError('supplemental manifest escapes its directory')
    manifest = read_json(manifest_path, 64*1024)
    if (type(manifest.get('schema')) is not int or manifest['schema'] != 1 or manifest.get('kind') != KIND or
            manifest.get('guardMode') != 'esp32' or manifest.get('pcMonitor') != 'paused' or
            manifest.get('isRequiredTestRigValidation') is not False or
            manifest.get('requiredCiChecksChanged') is not False):
        raise ValueError('supplemental manual-guard manifest schema is invalid')
    if (repository.lower() != 'researchanddesire/ossm' or
            str(manifest.get('repository')).lower() != repository.lower() or
            type(manifest.get('pr')) is not int or manifest['pr'] != int(pr)):
        raise ValueError('supplemental repository or PR does not match this run')
    if (not isinstance(head_sha, str) or not SHA.fullmatch(head_sha) or
            manifest.get('headSha') != head_sha):
        raise ValueError('supplemental firmware SHA does not match the current PR head')
    video = asset(root, manifest.get('video'), '.mp4', 512*1024*1024)
    with video.open('rb') as source:
        if source.read(12)[4:8] != b'ftyp':
            raise ValueError('supplemental video is not an MP4 container')
    controller_path = asset(root, manifest.get('controller'), '.json', 16*1024*1024)
    controller = read_json(controller_path, 16*1024*1024)
    identity = controller.get('identity') or {}
    if (not isinstance(identity, dict) or controller.get('expectedBuild') != head_sha or identity.get('build') != head_sha or
            identity.get('model') != 'OSSM' or type(controller.get('completed')) is not bool):
        raise ValueError('supplemental runtime identity or expected firmware SHA does not match')
    return {'headSha': head_sha, 'video': video, 'controller': controller,
            'controllerSha256': manifest['controller']['sha256'],
            'videoSha256': manifest['video']['sha256']}


def manual_section(evidence, media, run_url):
    if not evidence:
        return []
    controller = evidence['controller']
    lines = ['', '### Supplemental manual ESP32 guard validation', '',
             f"PR head `{evidence['headSha']}`; runtime identity and expected build match this head.",
             'The ESP32 onboard guard protected this manual run; the PC RS-485 monitor was paused.',
             'This supplemental evidence does not satisfy or change the required Test Rig checks.', '',
             '- Controller completed: '+str(controller['completed']).lower(),
             f"- Original recording SHA-256: `{evidence['videoSha256']}`",
             f"- Controller JSON SHA-256: `{evidence['controllerSha256']}`"]
    home = controller.get('home')
    if isinstance(home, dict):
        contacts, bounds, source = home.get('contacts'), home.get('boundsValid'), home.get('positionSource')
        lines += ['- Reported homing contacts: '+str(contacts if type(contacts) is int and 0 <= contacts <= 2 else 'unknown'),
                  '- Reported encoder bounds valid: '+(str(bounds).lower() if type(bounds) is bool else 'unknown'),
                  '- Reported position source: '+(source if source in ('rs485', 'stepper') else 'unknown')]
    if media:
        lines += ['', f"[Manual run MP4, {media['length']:.0f} s]({media['mp4']})", '',
                  f"<img src=\"{media['gif']}\" alt=\"Supplemental manual ESP32 guard validation\">",
                  f"Preview covers the whole recording at {media['speed']:.0f}× speed."]
    else:
        lines += ['', 'Manual video could not be published; the original recording remains in the artifact.']
    lines += ['', f'Original MP4, controller JSON and manual manifest remain in the '
              f'`hardware-evidence` artifact of the [workflow run]({run_url}). '
              'JSON is not uploaded to the media-only storage bucket.']
    return lines
