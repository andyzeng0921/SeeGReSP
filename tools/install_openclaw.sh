#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
OPENCLAW_DIR="$ROOT/third_party/openclaw"
OPENCLAW_REPOSITORY="https://github.com/openclaw/openclaw.git"
OPENCLAW_COMMIT="d0669429d857e4cd2cfc39b030095c5424f0b94f"

mkdir -p "$ROOT/third_party"
if [[ ! -d "$OPENCLAW_DIR/.git" ]]; then
  if [[ -e "$OPENCLAW_DIR" ]]; then
    echo "Refusing to replace non-Git path: $OPENCLAW_DIR" >&2
    exit 2
  fi
  git clone --filter=blob:none "$OPENCLAW_REPOSITORY" "$OPENCLAW_DIR"
fi

origin_url="$(git -C "$OPENCLAW_DIR" remote get-url origin 2>/dev/null || true)"
if [[ "$origin_url" != "$OPENCLAW_REPOSITORY" ]]; then
  echo "Refusing unverified OpenClaw origin: ${origin_url:-<missing>}" >&2
  exit 2
fi
if [[ -n "$(git -C "$OPENCLAW_DIR" status --porcelain)" ]]; then
  echo "OpenClaw source has local changes; refusing to change its revision." >&2
  exit 2
fi
if ! git -C "$OPENCLAW_DIR" cat-file -e "$OPENCLAW_COMMIT^{commit}" 2>/dev/null; then
  git -C "$OPENCLAW_DIR" fetch --depth 1 origin "$OPENCLAW_COMMIT"
fi
git -C "$OPENCLAW_DIR" checkout --detach "$OPENCLAW_COMMIT"

set +u
export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
source "$NVM_DIR/nvm.sh"
nvm install 22.23.1
nvm use 22.23.1 >/dev/null
set -u
cd "$OPENCLAW_DIR"
if [[ ! -d node_modules ]]; then
  corepack pnpm install --frozen-lockfile
fi
corepack pnpm build
mkdir -p "$ROOT/runtime/openclaw-state"
if [[ ! -f "$ROOT/agent_workspace/openclaw.json" ]]; then
  install -m 600 \
    "$ROOT/agent_workspace/openclaw.example.json" \
    "$ROOT/agent_workspace/openclaw.json"
  echo "Created ignored OpenClaw config; review its workspace path before use."
fi
corepack pnpm openclaw --version
