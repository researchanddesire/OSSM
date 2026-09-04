# AGENTS.md

## Assembly content synchronization

This repository (`researchanddesire/OSSM`, not `__ossm`) is the English-only OSSM
spoke of the rad-app hub. `assembly-docs/**` mirrors only
`rad-app/apps/docs-ohai/content/docs/ossm/**`. Hub `page.en.mdx` becomes
`page.mdx` here; `meta.en.json` becomes `meta.json`. Shared product assets,
KiCad sources, and any `_bom/` snapshots travel unchanged. Do not add translations
or other products here; all locales live in the hub and assembly-docs full mirror.

The notifier wakes rad-app after content changes. Reviewed sync PRs import to
hub staging, then fan out to the full mirror. Never dispatch directly to
assembly-docs or write another spoke. `.github/assembly-sync.json` is the
hub-managed baseline; do not hand-edit it. Initial content seeding requires a
separate reviewed hub proposal. Main follows normal staging-to-main promotion.
The notifier uses `RAD_VERSION_CONTROL_APP_ID` and
`RAD_VERSION_CONTROL_PRIVATE_KEY` with a token scoped only to rad-app.

The historical `Documentation/` folder is not part of content synchronization.
It may remain as reference material; CI must not reject its presence. The hub
and `assembly-docs/` are the active assembly publication sources.

## Branch and Release Policy

- Persistent branches are lowercase `staging` and `main`.
- Fetch before branching. Start feature and release work from `origin/staging`, then open the pull request back into `staging`.
- Only urgent production bug fixes may target `main` directly. Start them from `origin/main`, open the pull request into `main`, and make the standalone word `hotfix` the first word of the PR title (case-insensitive).
- Promote normal releases with a same-repository `staging` to `main` pull request only after every required build, unit, integration, artifact-verification, and OSSM hardware validation succeeds.
- Immediately merge every main hotfix back into `staging` with a `main` to `staging` pull request.
- Squash feature and hotfix pull requests into one focused commit with a concise, imperative title. Merge long-lived branch synchronization pull requests (`staging` to `main` and `main` to `staging`) with a merge commit; never squash or rebase them.
- Use matching branch names and linked pull requests for changes spanning RAD App or another firmware repository.
- Staging firmware reports the `staging` track and checks `https://staging.researchanddesire.com`. Main firmware reports `main` and checks `https://dashboard.researchanddesire.com`.
- Firmware is update-eligible only after its immutable Supabase artifacts and all required validation records verify. Missing hardware runners leave a release in `validating`; never bypass a gate.

## Development Safety

- Preserve the dedicated TLS/update task and MQTT pause behavior. Keep update protocol, checksum, reboot, and rollback behavior covered by native tests.
- Never commit credentials, local build products, or generated secrets.
