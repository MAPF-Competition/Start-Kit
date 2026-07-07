#!/usr/bin/env bash
set -euo pipefail

fix_format=false
tidy_fail=false
tidy_jobs=1
tidy_option_seen=false

usage() {
    cat <<'EOF'
Usage: ./lint_cpp_style.sh [options]

Runs the local C++ style pipeline:
  1. check clang-format
  2. configure build-style compile commands
  3. report clang-tidy naming diagnostics

Options:
  --fix-format       Apply clang-format in place, then exit.
  --tidy-fail        Return non-zero if clang-tidy reports diagnostics.
  --tidy-jobs N      Run up to N clang-tidy jobs in parallel. Default: 1.
  -j N               Short form of --tidy-jobs N.
  -h, --help         Show this help.

Examples:
  ./lint_cpp_style.sh
  ./lint_cpp_style.sh --fix-format
  ./lint_cpp_style.sh --tidy-jobs 8
  ./lint_cpp_style.sh --tidy-fail --tidy-jobs 8
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fix-format)
            fix_format=true
            ;;
        --tidy-fail)
            tidy_fail=true
            tidy_option_seen=true
            ;;
        --tidy-jobs)
            if [[ $# -lt 2 ]]; then
                echo "--tidy-jobs requires a positive integer." >&2
                exit 2
            fi
            tidy_jobs="$2"
            tidy_option_seen=true
            shift
            ;;
        --tidy-jobs=*)
            tidy_jobs="${1#*=}"
            tidy_option_seen=true
            ;;
        -j)
            if [[ $# -lt 2 ]]; then
                echo "-j requires a positive integer." >&2
                exit 2
            fi
            tidy_jobs="$2"
            tidy_option_seen=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if ! [[ "$tidy_jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo "--tidy-jobs must be a positive integer, got: $tidy_jobs" >&2
    exit 2
fi

if [[ "$fix_format" == true && "$tidy_option_seen" == true ]]; then
    echo "--fix-format only applies formatting and cannot be combined with clang-tidy options." >&2
    exit 2
fi

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

find_tool() {
    local preferred="$1"
    local fallback="$2"

    if command -v "$preferred" >/dev/null 2>&1; then
        command -v "$preferred"
    elif command -v "$fallback" >/dev/null 2>&1; then
        command -v "$fallback"
    else
        echo "Required tool not found: $preferred or $fallback" >&2
        exit 1
    fi
}

clang_format="$(find_tool clang-format-14 clang-format)"
if [[ "$fix_format" == false ]]; then
    clang_tidy="$(find_tool clang-tidy-14 clang-tidy)"
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT

cpp_files="$tmp_dir/cpp-files.txt"

echo "==> Collecting C++ files"
git ls-files \
    '*.cpp' '*.cc' '*.cxx' '*.h' '*.hpp' '*.hh' \
    ':!:build/**' \
    ':!:build-style/**' \
    ':!:third_party/**' \
    ':!:external/**' \
    ':!:generated/**' \
    ':!:inc/nlohmann/**' > "$cpp_files"

if [[ ! -s "$cpp_files" ]]; then
    echo "No C++ files found." >&2
    exit 1
fi

if [[ "$fix_format" == true ]]; then
    echo "==> Applying clang-format with $clang_format"
    xargs "$clang_format" -i < "$cpp_files"
    echo "==> Format fix complete"
    exit 0
else
    echo "==> Checking clang-format with $clang_format"
    xargs "$clang_format" --dry-run --Werror < "$cpp_files"
fi

cpp_tidy_files="$tmp_dir/cpp-tidy-files.txt"
git ls-files \
    '*.cpp' '*.cc' '*.cxx' \
    ':!:build/**' \
    ':!:build-style/**' \
    ':!:third_party/**' \
    ':!:external/**' \
    ':!:generated/**' \
    ':!:inc/nlohmann/**' > "$cpp_tidy_files"

if [[ ! -s "$cpp_tidy_files" ]]; then
    echo "No C++ translation units found." >&2
    exit 1
fi

cmake_args=(
    -S .
    -B build-style
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    -DBUILD_TESTS=ON
)

if [[ -n "${pybind11_DIR:-}" ]]; then
    cmake_args+=("-Dpybind11_DIR=$pybind11_DIR")
elif [[ -f "/home/duo/miniforge3/envs/lorr/share/cmake/pybind11/pybind11Config.cmake" ]]; then
    cmake_args+=("-Dpybind11_DIR=/home/duo/miniforge3/envs/lorr/share/cmake/pybind11")
else
    pybind11_cmake_dir=""
    if command -v python >/dev/null 2>&1; then
        pybind11_cmake_dir="$(python -m pybind11 --cmakedir 2>/dev/null || true)"
    fi
    if [[ -n "$pybind11_cmake_dir" ]]; then
        cmake_args+=("-Dpybind11_DIR=$pybind11_cmake_dir")
    fi
fi

echo "==> Configuring compile commands"
cmake "${cmake_args[@]}"

echo "==> Running clang-tidy naming diagnostics with $clang_tidy"
tidy_status=0
project_header_filter='(^|.*/)(default_planner|python/common|src|tests)/.*|(^|.*/)inc/[^/]+\.(h|hpp|hh)$'
tidy_args=(-p build-style "--header-filter=$project_header_filter" --quiet --extra-arg=-w)
if [[ "$tidy_fail" == true ]]; then
    tidy_args+=(--warnings-as-errors=readability-identifier-naming)
fi

filter_tidy_output() {
    sed -E '/^[0-9]+ warnings? generated\.$/d; /^Suppressed [0-9]+ warnings?/d'
}

if [[ "$tidy_jobs" -eq 1 ]]; then
    while IFS= read -r file; do
        set +e
        "$clang_tidy" "$file" "${tidy_args[@]}" 2>&1 | filter_tidy_output
        clang_tidy_status=${PIPESTATUS[0]}
        set -e
        if [[ "$clang_tidy_status" -ne 0 ]]; then
            tidy_status="$clang_tidy_status"
        fi
    done < "$cpp_tidy_files"
else
    echo "==> Using $tidy_jobs parallel clang-tidy jobs"
    export CLANG_TIDY="$clang_tidy"
    set +e
    xargs -r -P "$tidy_jobs" -I {} bash -c '
        file="$1"
        shift
        "$CLANG_TIDY" "$file" "$@" 2>&1 \
            | sed -E "/^[0-9]+ warnings? generated\.$/d; /^Suppressed [0-9]+ warnings?/d"
        exit "${PIPESTATUS[0]}"
    ' bash "{}" "${tidy_args[@]}" < "$cpp_tidy_files"
    tidy_status=$?
    set -e
fi

if [[ "$tidy_status" -ne 0 ]]; then
    if [[ "$tidy_fail" == true ]]; then
        echo "clang-tidy reported diagnostics and --tidy-fail is enabled." >&2
        exit "$tidy_status"
    fi
    echo "clang-tidy reported diagnostics; continuing because naming is advisory by default."
fi

echo "==> C++ style pipeline complete"
