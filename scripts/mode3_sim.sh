#!/usr/bin/env bash
# Usage: scripts/mode3_sim.sh [--text] [--no-checks]
set -u
set -o pipefail

# Share checks, logging and ordered shutdown with the real-chair entry point.
# shellcheck source=scripts/mode3_real.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/mode3_real.sh" || exit 1
mode3_main sim "$@"
