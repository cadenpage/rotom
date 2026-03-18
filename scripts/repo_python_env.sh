#!/bin/bash
set -eo pipefail

# Run local helper scripts with a repo-local venv when present, otherwise system
# Python, while exposing the repo's source tree.
export PYTHONPATH=/home/caden/Documents/rotom/src${PYTHONPATH:+:$PYTHONPATH}

PYTHON_BIN=/usr/bin/python3
if [ -x /home/caden/Documents/rotom/.venv/bin/python ]; then
  PYTHON_BIN=/home/caden/Documents/rotom/.venv/bin/python
fi

exec "$PYTHON_BIN" "$@"
