#!/usr/bin/env bash
# Load a config/profiles/*.env then exec a command.
#   ./scripts/run_with_profile.sh edge-brain-lean python -m examples.run_service
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PROFILE="${1:?usage: $0 <profile-name> <command...>}"
shift
ENV_FILE="${ROOT}/config/profiles/${PROFILE}.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "missing profile: ${ENV_FILE}" >&2
  exit 1
fi
set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a
cd "$ROOT"
exec "$@"
