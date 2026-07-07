#!/usr/bin/env bash
set -euo pipefail

fix_format=false
tidy_only=false
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
  --tidy-only        Skip clang-format and only run clang-tidy.
  --tidy-jobs N      Run up to N clang-tidy jobs in parallel. Default: 1.
  -j N               Short form of --tidy-jobs N.
  -h, --help         Show this help.

Examples:
  ./lint_cpp_style.sh
  ./lint_cpp_style.sh --fix-format
  ./lint_cpp_style.sh --tidy-only --tidy-jobs 8
  ./lint_cpp_style.sh --tidy-jobs 8
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fix-format)
            fix_format=true
            ;;
        --tidy-only)
            tidy_only=true
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

if [[ "$tidy_only" == false ]]; then
    clang_format="$(find_tool clang-format-14 clang-format)"
fi
if [[ "$fix_format" == false ]]; then
    clang_tidy="$(find_tool clang-tidy-14 clang-tidy)"
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT

if [[ "$tidy_only" == false ]]; then
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
        set +e
        xargs "$clang_format" --dry-run --Werror < "$cpp_files"
        format_status=$?
        set -e
        if [[ "$format_status" -ne 0 ]]; then
            echo "clang-format check failed. Run ./lint_cpp_style.sh --fix-format and commit the result." >&2
            exit 1
        fi
    fi
else
    echo "==> Skipping clang-format (--tidy-only)"
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
tidy_args=(
    -p build-style
    "--header-filter=$project_header_filter"
    --quiet
    --extra-arg=-w
    "--warnings-as-errors=*"
)

filter_tidy_output() {
    sed -E '/^[0-9]+ warnings? generated\.$/d; /^Suppressed [0-9]+ warnings?/d'
}

dedupe_tidy_output() {
    awk '
        /^[^[:space:]].*:[0-9]+:[0-9]+: (warning|error): / {
            if (seen[$0]++) {
                skip = 1
                next
            }
            skip = 0
            print
            next
        }
        /^(Error while processing|Found compiler error)/ {
            skip = 0
            print
            next
        }
        {
            if (!skip) {
                print
            }
        }
    '
}

print_tidy_report() {
    local report_file="$1"
    local checked_files="$2"
    local diagnostic_files="$tmp_dir/clang-tidy-diagnostic-files.txt"
    local error_count
    local warning_count
    local file_count
    local checked_count

    error_count="$(awk '/^[^[:space:]].*:[0-9]+:[0-9]+: error: / {count++} END {print count + 0}' "$report_file")"
    warning_count="$(awk '/^[^[:space:]].*:[0-9]+:[0-9]+: warning: / {count++} END {print count + 0}' "$report_file")"
    awk -v repo_root="$repo_root" '
        /^[^[:space:]].*:[0-9]+:[0-9]+: (warning|error): / {
            file = $0
            sub(/:[0-9]+:[0-9]+: (warning|error): .*/, "", file)
            if (index(file, repo_root "/") == 1) {
                file = substr(file, length(repo_root) + 2)
            }
            print file
        }
    ' "$report_file" | sort -u > "$diagnostic_files"

    file_count="$(wc -l < "$diagnostic_files" | tr -d ' ')"
    checked_count="$(wc -l < "$checked_files" | tr -d ' ')"

    echo "==> clang-tidy summary"
    echo "Translation units checked: $checked_count"
    echo "Errors: $error_count"
    echo "Warnings: $warning_count"
    echo "Files with diagnostics: $file_count"
    if [[ "$file_count" -gt 0 ]]; then
        echo "Files:"
        sed 's/^/  /' "$diagnostic_files"
    fi
}

emit_github_annotations() {
    local report_file="$1"

    if [[ "${GITHUB_ACTIONS:-}" != "true" ]]; then
        return 0
    fi

    awk -v repo_root="$repo_root" '
        function escape_data(value) {
            gsub(/%/, "%25", value)
            gsub(/\r/, "%0D", value)
            gsub(/\n/, "%0A", value)
            return value
        }
        function escape_property(value) {
            value = escape_data(value)
            gsub(/:/, "%3A", value)
            gsub(/,/, "%2C", value)
            return value
        }
        /^[^[:space:]].*:[0-9]+:[0-9]+: (warning|error): / {
            split($0, parts, ":")
            file = parts[1]
            line = parts[2]
            col = parts[3]
            message = $0
            sub(/^[^:]+:[0-9]+:[0-9]+: (warning|error): /, "", message)
            if (index(file, repo_root "/") == 1) {
                file = substr(file, length(repo_root) + 2)
            }
            printf "::error file=%s,line=%s,col=%s,title=clang-tidy::%s\n",
                escape_property(file), line, col, escape_data(message)
        }
    ' "$report_file"
}

write_github_step_summary() {
    local report_file="$1"
    local checked_files="$2"
    local diagnostic_files="$tmp_dir/clang-tidy-summary-files.txt"
    local error_count
    local warning_count
    local file_count
    local checked_count

    if [[ -z "${GITHUB_STEP_SUMMARY:-}" ]]; then
        return 0
    fi

    error_count="$(awk '/^[^[:space:]].*:[0-9]+:[0-9]+: error: / {count++} END {print count + 0}' "$report_file")"
    warning_count="$(awk '/^[^[:space:]].*:[0-9]+:[0-9]+: warning: / {count++} END {print count + 0}' "$report_file")"
    awk -v repo_root="$repo_root" '
        /^[^[:space:]].*:[0-9]+:[0-9]+: (warning|error): / {
            file = $0
            sub(/:[0-9]+:[0-9]+: (warning|error): .*/, "", file)
            if (index(file, repo_root "/") == 1) {
                file = substr(file, length(repo_root) + 2)
            }
            print file
        }
    ' "$report_file" | sort -u > "$diagnostic_files"

    file_count="$(wc -l < "$diagnostic_files" | tr -d ' ')"
    checked_count="$(wc -l < "$checked_files" | tr -d ' ')"

    {
        echo "## C++ Style Report"
        echo
        echo "| Metric | Count |"
        echo "| --- | ---: |"
        echo "| Translation units checked | $checked_count |"
        echo "| Errors | $error_count |"
        echo "| Warnings | $warning_count |"
        echo "| Files with diagnostics | $file_count |"
        if [[ "$file_count" -gt 0 ]]; then
            echo
            echo "### Files With Diagnostics"
            while IFS= read -r diagnostic_file; do
                echo "- \`$diagnostic_file\`"
            done < "$diagnostic_files"
        fi
    } >> "$GITHUB_STEP_SUMMARY"
}

tidy_output="$tmp_dir/clang-tidy-output.txt"
deduped_tidy_output="$tmp_dir/clang-tidy-deduped-output.txt"
if [[ "$tidy_jobs" -eq 1 ]]; then
    : > "$tidy_output"
    while IFS= read -r file; do
        set +e
        "$clang_tidy" "$file" "${tidy_args[@]}" >> "$tidy_output" 2>&1
        clang_tidy_status=$?
        set -e
        if [[ "$clang_tidy_status" -ne 0 ]]; then
            tidy_status="$clang_tidy_status"
        fi
    done < "$cpp_tidy_files"
    filter_tidy_output < "$tidy_output" | dedupe_tidy_output > "$deduped_tidy_output"
else
    echo "==> Using $tidy_jobs parallel clang-tidy jobs"
    tidy_logs_dir="$tmp_dir/clang-tidy-logs"
    tidy_indexed_files="$tmp_dir/cpp-tidy-indexed-files.txt"
    mkdir "$tidy_logs_dir"
    awk '{printf "%06d:%s\n", NR, $0}' "$cpp_tidy_files" > "$tidy_indexed_files"
    export CLANG_TIDY="$clang_tidy"
    export TIDY_LOGS_DIR="$tidy_logs_dir"
    set +e
    xargs -r -P "$tidy_jobs" -I {} bash -c '
        entry="$1"
        shift
        index="${entry%%:*}"
        file="${entry#*:}"
        "$CLANG_TIDY" "$file" "$@" > "$TIDY_LOGS_DIR/$index.log" 2>&1
    ' bash "{}" "${tidy_args[@]}" < "$tidy_indexed_files"
    tidy_status=$?
    set -e
    for log_file in "$tidy_logs_dir"/*.log; do
        filter_tidy_output < "$log_file"
    done | dedupe_tidy_output > "$deduped_tidy_output"
fi

cat "$deduped_tidy_output"
print_tidy_report "$deduped_tidy_output" "$cpp_tidy_files"
write_github_step_summary "$deduped_tidy_output" "$cpp_tidy_files"
emit_github_annotations "$deduped_tidy_output"

if [[ "$tidy_status" -ne 0 ]]; then
    echo "clang-tidy reported diagnostics." >&2
    exit 1
fi

echo "==> C++ style pipeline complete"
