#!/usr/bin/env bash
# The bench runner's job-started hook (specs/bench-ci.md section 2, item 2). The runner's `.env`
# sets ACTIONS_RUNNER_HOOK_JOB_STARTED to this script, and the runner runs it before every job; a
# non-zero exit aborts the job before any of its steps.
#
# A job is allowed when the repository is this one and either:
#   - the event is push, schedule or workflow_dispatch and the actor is the owner; or
#   - the event is pull_request, the PR's head repository is this repository, and the actor is the
#     owner or dependabot[bot].
# Everything else is refused, a PR from a fork among it, approved or not.
#
# The copy that runs is the one installed with the runner, outside its work directory, so a PR
# cannot change it. This one in the repository is public on purpose: the guard does not depend on
# being secret. It reads only the environment and the event file, with python3 (no jq needed).
set -u

REPO="hoverboardhavoc/hoverboard-firmware"
OWNER="hoverboardhavoc"

refuse() {
  echo "bench runner: refused: $*" >&2
  exit 1
}

repo="${GITHUB_REPOSITORY:-}"
event="${GITHUB_EVENT_NAME:-}"
actor="${GITHUB_ACTOR:-}"

[ "$repo" = "$REPO" ] || refuse "repository '$repo' is not $REPO"

case "$event" in
  push|schedule|workflow_dispatch)
    [ "$actor" = "$OWNER" ] || refuse "$event by '$actor', not the owner"
    ;;
  pull_request)
    path="${GITHUB_EVENT_PATH:-}"
    [ -n "$path" ] && [ -r "$path" ] || refuse "pull_request with no readable event file"
    head="$(python3 -c '
import json, sys
try:
    e = json.load(open(sys.argv[1]))
    print(e["pull_request"]["head"]["repo"]["full_name"])
except Exception:
    print("")
' "$path")"
    [ "$head" = "$REPO" ] || refuse "pull_request from head repository '$head' (a fork?)"
    case "$actor" in
      "$OWNER"|"dependabot[bot]") ;;
      *) refuse "pull_request by '$actor'" ;;
    esac
    ;;
  *)
    refuse "event '$event'"
    ;;
esac

echo "bench runner: allowed: $event by $actor on $repo"
exit 0
