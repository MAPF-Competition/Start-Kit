#!/usr/bin/env bash
set -euo pipefail

fix_format=false
tidy_fail=false

usage() {
    cat <<'EOF'
Usage: ./lint_cpp_style.sh [--fix-format] [--tidy-fail]

Runs the local C++ style pipeline:
  1. check or fix clang-format
  2. configure build-style compile commands
  3. report clang-tidy naming diagnostics

Options:
  --fix-format  Apply clang-format in place before running clang-tidy.
  --tidy-fail   Return non-zero if clang-tidy reports diagnostics.
  -h, --help    Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fix-format)
            fix_format=true
            ;;
        --tidy-fail)
            tidy_fail=true
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
clang_tidy="$(find_tool clang-tidy-14 clang-tidy)"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT

cpp_files="$tmp_dir/cpp-files.txt"
cpp_tidy_files="$tmp_dir/cpp-tidy-files.txt"

echo "==> Collecting C++ files"
git ls-files \
    '*.cpp' '*.cc' '*.cxx' '*.h' '*.hpp' '*.hh' \
    ':!:build/**' \
    ':!:build-style/**' \
    ':!:third_party/**' \
    ':!:external/**' \
    ':!:generated/**' \
    ':!:inc/nlohmann/**' > "$cpp_files"

git ls-files \
    '*.cpp' '*.cc' '*.cxx' \
    ':!:build/**' \
    ':!:build-style/**' \
    ':!:third_party/**' \
    ':!:external/**' \
    ':!:generated/**' \
    ':!:inc/nlohmann/**' > "$cpp_tidy_files"

if [[ ! -s "$cpp_files" ]]; then
    echo "No C++ files found." >&2
    exit 1
fi

if [[ ! -s "$cpp_tidy_files" ]]; then
    echo "No C++ translation units found." >&2
    exit 1
fi

if [[ "$fix_format" == true ]]; then
    echo "==> Applying clang-format with $clang_format"
    xargs "$clang_format" -i < "$cpp_files"
else
    echo "==> Checking clang-format with $clang_format"
    xargs "$clang_format" --dry-run --Werror < "$cpp_files"
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
tidy_args=(-p build-style --quiet --extra-arg=-w)
if [[ "$tidy_fail" == true ]]; then
    tidy_args+=(--warnings-as-errors=readability-identifier-naming)
fi

while IFS= read -r file; do
    set +e
    "$clang_tidy" "$file" "${tidy_args[@]}" 2>&1 \
        | sed -E '/^[0-9]+ warnings? generated\.$/d; /^Suppressed [0-9]+ warnings?/d'
    clang_tidy_status=${PIPESTATUS[0]}
    set -e
    if [[ "$clang_tidy_status" -ne 0 ]]; then
        tidy_status="$clang_tidy_status"
    fi
done < "$cpp_tidy_files"

if [[ "$tidy_status" -ne 0 ]]; then
    if [[ "$tidy_fail" == true ]]; then
        echo "clang-tidy reported diagnostics and --tidy-fail is enabled." >&2
        exit "$tidy_status"
    fi
    echo "clang-tidy reported diagnostics; continuing because naming is advisory by default."
fi

echo "==> C++ style pipeline complete"
