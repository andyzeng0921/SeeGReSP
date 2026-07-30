#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
OPENCLAW="$ROOT/third_party/openclaw"
WORKSPACE="$ROOT/agent_workspace"
export OPENCLAW_STATE_DIR="$ROOT/runtime/openclaw-state"
export OPENCLAW_CONFIG_PATH="$ROOT/agent_workspace/openclaw.json"

if [[ ! -f "$OPENCLAW_CONFIG_PATH" ]]; then
  echo "OpenClaw config is missing. Copy agent_workspace/openclaw.example.json" >&2
  echo "to agent_workspace/openclaw.json and review the workspace path first." >&2
  exit 2
fi

set +u
export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
source "$NVM_DIR/nvm.sh"
nvm use 22.23.1 >/dev/null
set -u
cd "$OPENCLAW"
if [[ ! -d node_modules ]]; then
  echo "OpenClaw dependencies are not installed. Run tools/install_openclaw.sh." >&2
  exit 2
fi

exec corepack pnpm openclaw agent exec \
  --cwd "$WORKSPACE" \
  --state-dir "$OPENCLAW_STATE_DIR" \
  --no-auth-env-only \
  "$@"
