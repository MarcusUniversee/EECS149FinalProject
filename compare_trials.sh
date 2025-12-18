#!/usr/bin/env bash
set -u  # don't use -e; we want to continue on errors

ROOT="sim2real_results"
RESULTS_ROOT="sim2real_results"
PY="python"
SCRIPT="./sim2real_analysis/compare_trials.py"

# Walk every immediate subfolder in corrected_data/*
for folder in "$ROOT"/*/; do

  base="$(basename "$folder")"

  # Skip anything that starts with SIM
  if [[ "$base" == COMBINED* ]]; then
    continue
  fi

  # Strip .csv extension
  name_no_ext="${base:5}"

  # Skip if results folder already exists
  if [ -d "$RESULTS_ROOT/COMBINED_$name_no_ext" ]; then
    echo "SKIP (already processed): $name_no_ext"
    continue
  fi

  echo "Running: $PY $SCRIPT --name \"$name_no_ext\""
  $PY "$SCRIPT" --name "$name_no_ext"
  rc=$?

  # Skip on error (non-zero exit code)
  if [ $rc -ne 0 ]; then
    echo "  -> SKIP (error code $rc): $folder" >&2
    continue
  fi
done