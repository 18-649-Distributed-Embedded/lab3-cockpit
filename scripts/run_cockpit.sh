#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec python3 "$ROOT/src/cockpit.py" \
  --env "$ROOT/config/cockpit.env" \
  "$@"