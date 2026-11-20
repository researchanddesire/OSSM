"""Hosted queue client and immutable publisher provenance."""
import argparse
import io
import json
import os
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError

import firmware_queue as queue
from publish_immutable_firmware import validation_payload


def state(name):
    return {"state": name, "admitted": name == "running",
            "terminal": name in {"passed", "failed", "cancelled"}}


class QueueTests(unittest.TestCase):
    def run_action(self, action, replies):
        with patch.object(queue, "post", side_effect=replies) as post, patch.object(queue.time, "sleep"):
            queue.main(action)
            return post.call_count

    def test_wait_requires_admission(self):
        self.assertEqual(self.run_action("wait", [state("queued"), state("running")]), 2)
        with self.assertRaisesRegex(SystemExit, "no longer waiting"):
            self.run_action("wait", [state("cancelled")])

    def test_observe_polls_until_terminal_including_cancellation(self):
        for end in ("passed", "failed", "cancelled"):
            with self.subTest(end=end):
                self.assertEqual(self.run_action("observe", [state("running"), state(end)]), 2)

    def test_sweep_reports_the_bounded_server_result_without_polling(self):
        self.assertEqual(self.run_action("sweep", [{"checked": 3, "updated": 2}]), 1)

    def test_temporary_failure_retries_but_denied_request_aborts(self):
        unavailable = HTTPError("https://example.test", 503, "unavailable", {}, None)
        self.assertEqual(self.run_action("wait", [unavailable, state("running")]), 2)
        with self.assertRaisesRegex(SystemExit, "HTTP 403"):
            self.run_action("wait", [HTTPError("https://example.test", 403, "denied", {}, None)])
        with self.assertRaisesRegex(SystemExit, "eight attempts"):
            self.run_action("observe", [unavailable] * 8)

    def test_cleanup_sends_workflow_attempt_and_keeps_issuer_credentials_separate(self):
        opener = Mock()
        opener.open.side_effect = [io.BytesIO(b'{"value":"test-oidc"}'), io.BytesIO(b'{"state":"cancelled"}')]
        with patch.dict(os.environ, {
            "ACTIONS_ID_TOKEN_REQUEST_URL": "https://issuer.test/token?api-version=2",
            "ACTIONS_ID_TOKEN_REQUEST_TOKEN": "test-issuer-token",
            "GITHUB_TOKEN": "test-github", "TARGET_RUN_ID": "41", "TARGET_ATTEMPT": "2",
            "QUEUE_ENVIRONMENT": "staging",
        }, clear=True):
            self.assertEqual(queue.post(opener, "reconcile"), {"state": "cancelled"})
        issuer, target = [call.args[0] for call in opener.open.call_args_list]
        self.assertTrue(issuer.full_url.endswith("&audience=" + queue.AUDIENCE))
        self.assertEqual(issuer.get_header("Authorization"), "Bearer test-issuer-token")
        self.assertIsNone(issuer.get_header("X-github-token"))
        self.assertEqual(target.full_url, queue.ENDPOINTS["staging"] + "/api/internal/firmware/v1/pr-queue")
        self.assertEqual(target.get_header("Authorization"), "Bearer test-oidc")
        self.assertEqual(target.get_header("X-github-token"), "test-github")
        self.assertEqual(json.loads(target.data), {"action": "reconcile", "runId": 41, "runAttempt": 2})

    def test_publisher_evidence_binds_release_sha_pr_and_attempt(self):
        base = "https://github.com/researchanddesire/firmware/actions/runs/41"
        logs = {layer: f"{base}/job/101#step:{number}:1" for number, layer in
                enumerate(("build", "unit", "integration"), 5)}
        args = argparse.Namespace(device_type="ossm", hardware_variant="v2", build_sha="b" * 40)
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "researchanddesire/firmware", "GITHUB_RUN_ID": "41",
            "GITHUB_RUN_ATTEMPT": "2", "FIRMWARE_MERGED_PR": "53", "GITHUB_EVENT_NAME": "push",
            "FIRMWARE_VALIDATION_LOG_LINKS": json.dumps(logs)
        }, clear=True):
            for layer, url in logs.items():
                record = validation_payload("release-a", layer, "success", args)
                self.assertEqual((record["releaseId"], record["repositorySha"]), ("release-a", args.build_sha))
                self.assertEqual(record["workflowRunUrl"], url)
                if layer == "build":
                    self.assertEqual(record["summary"], {"mergedPr": 53, "event": "push"})
            del os.environ["FIRMWARE_MERGED_PR"]
            del os.environ["FIRMWARE_VALIDATION_LOG_LINKS"]
            os.environ["GITHUB_EVENT_NAME"] = "workflow_dispatch"
            manual = validation_payload("release-a", "build", "success", args)
            self.assertEqual(manual["workflowRunUrl"], base + "/attempts/2")
            self.assertEqual(manual["summary"], {"mergedPr": None, "event": "workflow_dispatch"})


    def test_legacy_v1_step_contains_all_three_checks(self):
        dedicated = "https://github.com/researchanddesire/OSSM/actions/runs/41/job/101#step:8:1"
        args = argparse.Namespace(device_type="ossm", hardware_variant="v1", build_sha="a" * 40)
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "researchanddesire/OSSM", "GITHUB_RUN_ID": "41",
            "FIRMWARE_VALIDATION_LOG_LINKS": json.dumps({"v1": dedicated, "build": "v2-build",
                "unit": "v2-tests", "integration": "v2-installer"}),
        }, clear=True):
            for layer in ("build", "unit", "integration"):
                self.assertEqual(validation_payload("v1-release", layer, "success", args)["workflowRunUrl"], dedicated)


if __name__ == "__main__":
    unittest.main()
