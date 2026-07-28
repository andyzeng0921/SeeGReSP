#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
set +u
source /home/ubuntu/.nvm/nvm.sh
nvm install 22.23.1
nvm use 22.23.1 >/dev/null
set -u
cd "$ROOT/third_party/openclaw"
if [[ ! -d node_modules ]]; then
  corepack pnpm install --frozen-lockfile
fi
corepack pnpm build
mkdir -p "$ROOT/runtime/openclaw-state"
corepack pnpm openclaw --version
