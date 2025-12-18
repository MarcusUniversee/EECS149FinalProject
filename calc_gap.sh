#!/usr/bin/env bash
set -u  # don't use -e; we want to continue on errors

ROOT="corrected_data"
RESULTS_ROOT="sim2real_results"
PY="python"
SCRIPT="./sim2real_analysis/find_gap.py"

# Walk every immediate subfolder in corrected_data/*
for folder in "$ROOT"/*/; do
  # If no subfolders exist, skip
  [ -d "$folder" ] || continue

  # Iterate files in that subfolder
  for file in "$folder"*; do
    [ -f "$file" ] || continue

    base="$(basename "$file")"

    # Skip anything that starts with SIM
    if [[ "$base" == SIM* ]]; then
      continue
    fi

    # Strip .csv extension
    name_no_ext="${base%.csv}"

    # Skip if results folder already exists
    if [ -d "$RESULTS_ROOT/$name_no_ext" ]; then
      echo "SKIP (already processed): $name_no_ext"
      continue
    fi

    echo "Running: $PY $SCRIPT --folder \"$folder\" --name \"$base\""
    $PY "$SCRIPT" --folder "$folder" --name "$base"
    rc=$?

    # Skip on error (non-zero exit code)
    if [ $rc -ne 0 ]; then
      echo "  -> SKIP (error code $rc): $file" >&2
      continue
    fi
  done
done