# Contributing to Start-Kit

Thank you for helping improve Start-Kit. This guide describes the workflow for project
developers. Competition participants working in their private submission repositories should
continue to follow the submission documentation in the `README.md`.

## Before You Start

- Search existing issues and pull requests before starting duplicate work.
- Open an issue before making a substantial API, file-format, architecture, or behavior change.
- Keep each pull request focused on one change. Separate unrelated cleanup into another branch.
- Never commit credentials, local configuration, generated build output, or large result files.

## Branch Workflow

The protected branches have distinct roles:

- `main` contains released, stable code. Only a pull request from this repository's `dev` branch
  may target `main`.
- `dev` is the integration branch. Do not push directly to it.
- Feature and maintenance branches are created from the latest `dev` and merged back into `dev`
  through a pull request.

Create a branch:

```shell
git switch dev
git pull --ff-only origin dev
git switch -c feature/delay-aware-planner
```

Use a short, descriptive, lowercase name with hyphens:

- `feature/<name>` for new behavior
- `fix/<name>` for bug fixes
- `docs/<name>` for documentation
- `refactor/<name>` for behavior-preserving code changes
- `test/<name>` for test-only changes
- `chore/<name>` for maintenance

For example: `feature/delay-aware-planner`, `fix/task-validation`, or
`docs/python-interface`. Existing branches created before this convention do not need to be
renamed.

Before requesting final review, fetch `dev` and update your branch so the pull request is tested
against current integration code. Avoid rewriting branch history after review has started unless
you coordinate with the reviewers.

## C++ Style

The configuration files are the source of truth:

- `.clang-format` uses a Google-derived style with four-space indentation, no tabs, and a
  100-column limit.
- `.clang-tidy` requires `CamelCase` for classes, structs, and enums, and `lower_case` for
  functions and methods.

Some public callbacks, compatibility types, and existing legacy identifiers are explicitly
excluded from the naming rules. Do not rename those interfaces solely to satisfy a general naming
convention, and do not reuse a legacy spelling for new identifiers. Rules that are commented out
in `.clang-tidy` are deferred and are not currently enforced.

Use Clang 14 when possible so local results match GitHub Actions. On Ubuntu or Debian, install the
same primary tools and libraries used by CI:

```shell
sudo apt-get update
sudo apt-get install clang-format-14 clang-tidy-14 cmake g++ \
  libboost-filesystem-dev libboost-log-dev libboost-program-options-dev \
  libboost-system-dev pybind11-dev python3-dev
```

## Run the Style Checks

Fetch the current integration branch, format only the C++ lines changed by your contribution,
and run the complete contribution check:

```shell
git fetch origin dev
./lint_cpp_style.sh --fix-format --format-base origin/dev
./lint_cpp_style.sh --format-base origin/dev --tidy-jobs "$(nproc)"
```

The script performs these operations:

1. Checks formatting. With `--format-base REF`, it checks only C++ lines changed since the merge
   base with `REF`; without that option, it checks every tracked project C++ file.
2. Configures `build-style/compile_commands.json` with CMake.
3. Runs the `.clang-tidy` naming diagnostics across all project translation units.

Available options:

- `--fix-format` applies formatting and exits. Combine it with `--format-base REF` for the normal
  contribution workflow.
- `--format-base REF` limits formatting checks or fixes to lines changed by the contribution.
- `--tidy-only` skips formatting and runs only the naming diagnostics.
- `--tidy-jobs N` or `-j N` runs up to `N` clang-tidy processes in parallel.
- `--help` prints the current usage details.

Build directories, third-party code, generated code, and `inc/nlohmann/` are excluded. The full
repository format check may report legacy formatting debt, so pull requests currently enforce
formatting incrementally while enforcing naming across the complete project.

### Enable Automatic Checks Before Push

This repository includes a tracked `pre-push` hook. Git does not enable hooks from a cloned
repository automatically, so each developer must opt in once:

```shell
git config core.hooksPath .githooks
```

After it is enabled, `git push` runs the incremental formatting and project-wide naming checks
before Git transfers the commits. The hook also rejects direct pushes to `dev` and `main`. For an
exceptional troubleshooting push, Git allows hooks to be bypassed with `git push --no-verify`, but
GitHub Actions and protected-branch requirements remain authoritative.

## Build and Test

For code changes, build and run the complete test suite before requesting review:

```shell
./compile.sh
ctest --test-dir build --output-on-failure
```

Add or update tests for changed behavior. Update the relevant Markdown documentation when a
public option, interface, input, output, or developer workflow changes. Preserve documented
participant-facing callbacks and file formats unless the pull request explicitly proposes a
coordinated compatibility change.

## Commits and Pull Requests

- Write concise commit subjects in the imperative mood, such as `Add delay validation tests`.
- Explain why the change is needed, not only what files changed.
- Open the pull request against `dev`; only maintainers promote `dev` to `main`.
- Draft pull requests are welcome for early feedback, but mark the pull request ready only after
  local checks pass.
- Complete the pull request template, link related issues, and include reproducible testing
  evidence.
- Respond to review comments and resolve conversations only after the requested change or answer
  is present.

GitHub Actions repeats the style checks for every push and every pull request. Local checks provide
faster feedback, while the protected-branch checks are the authoritative merge requirement.
