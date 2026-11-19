#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
exec .venv/bin/python -B DAG_scheduler/run_combined.py "$@"
