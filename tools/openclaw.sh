#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
OPENCLAW="$ROOT/third_party/openclaw"
WORKSPACE="$ROOT/agent_workspace"
export OPENCLAW_STATE_DIR="$ROOT/runtime/openclaw-state"
export OPENCLAW_CONFIG_PATH="$ROOT/agent_workspace/openclaw.json"

set +u
source /home/ubuntu/.nvm/nvm.sh
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
