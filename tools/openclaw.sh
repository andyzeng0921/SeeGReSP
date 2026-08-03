#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
OPENCLAW="$ROOT/third_party/openclaw"
WORKSPACE="$ROOT/agent_workspace"
export OPENCLAW_STATE_DIR="$ROOT/runtime/openclaw-state"
export OPENCLAW_CONFIG_PATH="$ROOT/agent_workspace/openclaw.json"
mkdir -p "$ROOT/runtime"
NPMRC="$ROOT/runtime/openclaw-npmrc"
if [[ ! -e "$NPMRC" ]]; then
  install -m 600 /dev/null "$NPMRC"
fi
unset NPM_CONFIG_PREFIX npm_config_prefix NPM_CONFIG_GLOBALCONFIG npm_config_globalconfig
export NPM_CONFIG_USERCONFIG="$NPMRC"

export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
NODE_BIN="$NVM_DIR/versions/node/v22.23.1/bin"
if [[ ! -x "$NODE_BIN/node" || ! -x "$NODE_BIN/corepack" ]]; then
  echo "Pinned Node.js 22.23.1 is unavailable. Run tools/install_openclaw.sh." >&2
  exit 2
fi
export PATH="$NODE_BIN:$PATH"
cd "$OPENCLAW"
if [[ ! -d node_modules ]]; then
  echo "OpenClaw dependencies are not installed. Run tools/install_openclaw.sh." >&2
  exit 2
fi

agent_timeout=600
previous=""
for argument in "$@"; do
  if [[ "$previous" == "--timeout" && "$argument" =~ ^[1-9][0-9]*$ ]]; then
    agent_timeout="$argument"
  fi
  previous="$argument"
done
wall_timeout=$((agent_timeout + 15))

# OpenClaw may leave provider/tool subprocesses behind when its own deadline
# expires.  GNU timeout owns a separate process group and escalates to KILL so
# a failed validation turn cannot accumulate orphan agents.
exec timeout --signal=TERM --kill-after=5s "${wall_timeout}s" \
  corepack pnpm openclaw agent exec \
  --cwd "$WORKSPACE" \
  --state-dir "$OPENCLAW_STATE_DIR" \
  --no-auth-env-only \
  "$@"
