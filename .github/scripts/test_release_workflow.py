import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from release_workflow import branch_configuration, main, tag_action


class ReleaseWorkflowTests(unittest.TestCase):
    def test_release_writes_use_repository_scoped_app_tokens(self):
        workflows = Path(__file__).resolve().parents[1] / "workflows"
        for name in ["publish_firmware.yml", "hotfix-sync.yml"]:
            with self.subTest(workflow=name):
                workflow = (workflows / name).read_text()
                self.assertIn("uses: actions/create-github-app-token@v2", workflow)
                self.assertIn(
                    "app-id: ${{ vars.RAD_VERSION_CONTROL_APP_ID }}", workflow
                )
                self.assertIn(
                    "private-key: ${{ secrets.RAD_VERSION_CONTROL_PRIVATE_KEY }}",
                    workflow,
                )
                self.assertIn(
                    "repositories: ${{ github.event.repository.name }}", workflow
                )
                self.assertNotIn("RAD_VERSION_CONTROL_DEPLOY_KEY", workflow)
                self.assertNotIn("HOTFIX_SYNC_TOKEN", workflow)
                self.assertNotRegex(
                    workflow,
                    r"(?m)^\s+(?:token|github-token):\s+\$\{\{ github\.token \}\}$",
                )
        workflow = (workflows / "publish_firmware.yml").read_text()
        self.assertIn(
            "token: ${{ steps.version_token.outputs.token }}", workflow
        )
        self.assertIn(
            "github-token: ${{ steps.version_token.outputs.token }}", workflow
        )
        self.assertIn(
            "GH_TOKEN: ${{ steps.version_token.outputs.token }}",
            (workflows / "hotfix-sync.yml").read_text(),
        )

    def test_branch_configuration_maps_tracks_and_projects(self):
        self.assertEqual(branch_configuration("main")["PIO_ENV"], "production")
        self.assertEqual(branch_configuration("staging")["PIO_ENV"], "staging")
        self.assertNotEqual(
            branch_configuration("main")["STORAGE_PROJECT_REF"],
            branch_configuration("staging")["STORAGE_PROJECT_REF"],
        )
        with self.assertRaises(ValueError):
            branch_configuration("feature")

    def test_tag_action_is_idempotent_and_fails_on_collision(self):
        sha = "a" * 40
        self.assertEqual(tag_action("", sha), "create")
        self.assertEqual(tag_action(sha, sha), "exists")
        with self.assertRaisesRegex(ValueError, "different commit"):
            tag_action("b" * 40, sha)

    def test_configure_writes_github_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            env_file = Path(directory) / "env"
            with patch.dict("os.environ", {"GITHUB_ENV": str(env_file)}), patch(
                "sys.argv", ["release_workflow.py", "configure", "--branch", "staging"]
            ):
                self.assertEqual(main(), 0)
            self.assertIn("TRACK=staging", env_file.read_text())


if __name__ == "__main__":
    unittest.main()
