#!/usr/bin/env bash
set -euo pipefail

# Keep bootstrap and upgrader branch aligned when users pass --source-branch.
SOURCE_BRANCH="main"
for ((i=1; i<=$#; i++)); do
   arg="${!i}"
   if [[ "$arg" == "--source-branch" ]]; then
      next_index=$((i + 1))
      if (( next_index <= $# )); then
         SOURCE_BRANCH="${!next_index}"
      fi
   elif [[ "$arg" == --source-branch=* ]]; then
      SOURCE_BRANCH="${arg#--source-branch=}"
   fi
done

SCRIPT_URL="https://raw.githubusercontent.com/MAPF-Competition/Start-Kit/${SOURCE_BRANCH}/utils/upgrade_start_kit.py"
TMP_SCRIPT="$(mktemp "${PWD}/.upgrade_start_kit.XXXXXX.py")"

cleanup() {
   rm -f "$TMP_SCRIPT"
}
trap cleanup EXIT

if command -v curl >/dev/null 2>&1; then
   curl -fsSL -o "$TMP_SCRIPT" "$SCRIPT_URL"
elif command -v wget >/dev/null 2>&1; then
   wget -qO "$TMP_SCRIPT" "$SCRIPT_URL"
else
   echo "Error: neither curl nor wget is available to download the upgrader." >&2
   exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
   echo "Error: python3 is required to run the upgrader." >&2
   exit 1
fi

python3 "$TMP_SCRIPT" "$@"