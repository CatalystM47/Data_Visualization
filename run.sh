#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

if command -v python3 >/dev/null 2>&1; then PY=python3
elif command -v python >/dev/null 2>&1; then PY=python
else echo "[ERROR] Python not found. Install from https://www.python.org"; exit 1; fi

echo "=========================================================="
echo "  Drone Flight Log Analyzer - setup and run"
echo "=========================================================="

if [ ! -d ".venv" ]; then
    echo "[1/3] Creating virtual environment..."
    "$PY" -m venv .venv
fi

source .venv/bin/activate
echo "[2/3] Installing packages..."
python -m pip install --upgrade pip >/dev/null
python -m pip install -r requirements.txt

echo "[3/3] Starting analyzer..."
echo "Open http://localhost:10917 in your browser."
python app.py
