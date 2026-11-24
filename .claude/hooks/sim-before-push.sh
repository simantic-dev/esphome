#!/bin/sh
# Claude Code PreToolUse hook: before any `git push`, build sim/home.yaml and boot it on Simantic's simulated
# ESP32-C3 (sim/test.sh). Exit 2 blocks the push and hands the sim output back to Claude.
jq -r '.tool_input.command // ""' | grep -q 'git push' || exit 0
cd "${CLAUDE_PROJECT_DIR:-.}" || exit 0
[ -f venv/bin/activate ] && . venv/bin/activate
if ! out=$(sim/build.sh 2>&1); then
  printf 'Push blocked: firmware build failed.\n%s\n' "$(printf '%s\n' "$out" | tail -20)" >&2; exit 2
fi
if ! out=$(sim/test.sh 2>&1); then
  printf 'Push blocked: the firmware fails in the simulator (sim/test.sh):\n%s\nFull serial log: sim run --scenario sim/scenario.yaml --ascii\n' "$out" >&2; exit 2
fi
echo "sim/test.sh passed" >&2
