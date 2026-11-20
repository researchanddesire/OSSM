"""Hosted client for the shared Test Rig queue. Only GitHub-hosted jobs run this file."""
import json
import os
import sys
import time
import urllib.error
import urllib.request

ENDPOINTS = {
    "production": "https://dashboard.researchanddesire.com",
    "staging": "https://staging.researchanddesire.com",
}
AUDIENCE = "rad-firmware-validation-queue"


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("Unexpected redirect from the queue or token issuer")


def post(opener, action):
    issuer = os.environ["ACTIONS_ID_TOKEN_REQUEST_URL"]
    issuer += ("&" if "?" in issuer else "?") + "audience=" + AUDIENCE
    request = urllib.request.Request(issuer, headers={
        "Authorization": "Bearer " + os.environ["ACTIONS_ID_TOKEN_REQUEST_TOKEN"]
    })
    with opener.open(request, timeout=15) as response:
        oidc = json.load(response)["value"]
    payload = {"action": action}
    if action == "reconcile":
        payload.update(runId=int(os.environ["TARGET_RUN_ID"]), runAttempt=int(os.environ["TARGET_ATTEMPT"]))
    origin = ENDPOINTS[os.environ["QUEUE_ENVIRONMENT"]]
    request = urllib.request.Request(origin + "/api/internal/firmware/v1/pr-queue",
        data=json.dumps(payload).encode(), headers={
            "Content-Type": "application/json", "Authorization": "Bearer " + oidc,
            "X-GitHub-Token": os.environ["GITHUB_TOKEN"]
        }, method="POST")
    with opener.open(request, timeout=30) as response:
        return json.load(response)


def main(action):
    if action not in {"wait", "observe", "complete", "reconcile", "sweep"}:
        raise ValueError("Unknown queue operation")
    opener = urllib.request.build_opener(NoRedirect())
    previous = None
    failures = 0
    while True:
        try:
            result = post(opener, action)
            failures = 0
        except (urllib.error.URLError, TimeoutError) as error:
            if isinstance(error, urllib.error.HTTPError) and error.code < 500 and error.code not in (408, 429):
                raise SystemExit(f"Queue refused {action} (HTTP {error.code})") from None
            failures += 1
            if failures >= 8:
                raise SystemExit("Could not reach the validation queue after eight attempts") from None
            time.sleep(min(45, 5 * failures))
            continue
        if action == "sweep":
            print(f"Checked {int(result['checked'])} workflow attempts, updated {int(result['updated'])}", flush=True)
            return
        if previous != result["state"]:
            print("Queue state:", result["state"], flush=True)
            previous = result["state"]
        if action == "wait":
            if result["terminal"]:
                raise SystemExit("This workflow attempt is no longer waiting. Re-run all jobs to start again.")
            if result["admitted"]:
                return
        elif action != "observe" or result["terminal"]:
            return
        time.sleep(30)


if __name__ == "__main__":
    main(sys.argv[1])
