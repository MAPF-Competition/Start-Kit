#!/usr/bin/env python3
import argparse
import datetime as _dt
import json
import pathlib
import subprocess
import sys
import tempfile
import urllib.parse
import urllib.request

REMOTE_URL = "https://github.com/MAPF-Competition/Start-Kit.git"
RAW_BASE = "https://raw.githubusercontent.com/MAPF-Competition/Start-Kit"


def run(cmd, cwd=None, check=True, capture_output=True):
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=check,
        text=True,
        capture_output=capture_output,
    )


def git(args, repo_root, check=True):
    return run(["git", *args], cwd=repo_root, check=check)


def eprint(msg):
    print(msg, file=sys.stderr)


def get_repo_root():
    try:
        res = run(["git", "rev-parse", "--show-toplevel"], check=True)
    except subprocess.CalledProcessError:
        raise RuntimeError("This script must be run inside a git repository.")
    return pathlib.Path(res.stdout.strip())


def ensure_upstream(repo_root, remote_url):
    res = git(["remote"], repo_root)
    remotes = {x.strip() for x in res.stdout.splitlines() if x.strip()}
    if "upstream" in remotes:
        git(["remote", "set-url", "upstream", remote_url], repo_root)
    else:
        git(["remote", "add", "upstream", remote_url], repo_root)


def fetch_release(repo_root, version):
    tag = f"v{version}"
    git(["fetch", "upstream", "--no-tags", "main"], repo_root)
    git(["fetch", "upstream", "tag", tag, "--force"], repo_root)
    return tag


def load_manifest(repo_root, version, raw_ref=None, local_first=True):
    local_manifest = (
        repo_root
        / "utils"
        / "upgrades"
        / "releases"
        / version
        / "manifest.json"
    )
    ref_name = raw_ref or f"v{version}"
    encoded_ref = urllib.parse.quote(ref_name, safe="")
    url = f"{RAW_BASE}/{encoded_ref}/utils/upgrades/releases/{version}/manifest.json"

    def read_local_manifest():
        if local_manifest.exists():
            with local_manifest.open("r", encoding="utf-8") as f:
                return json.load(f), str(local_manifest)
        return None

    def read_remote_manifest():
        with urllib.request.urlopen(url) as r:
            return json.loads(r.read().decode("utf-8")), url

    if local_first:
        local = read_local_manifest()
        if local is not None:
            return local
        try:
            return read_remote_manifest()
        except Exception as ex:
            raise RuntimeError(
                f"Failed to load manifest for version {version}. Tried: {local_manifest} and {url}.\n{ex}"
            )

    try:
        return read_remote_manifest()
    except Exception as remote_ex:
        local = read_local_manifest()
        if local is not None:
            eprint(
                f"Warning: failed to load remote manifest ({url}). Falling back to local manifest."
            )
            return local
        raise RuntimeError(
            f"Failed to load manifest for version {version}. Tried: {url} and {local_manifest}.\n{remote_ex}"
        )


def dedupe_keep_order(items):
    seen = set()
    out = []
    for x in items:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out


def parse_semver(version):
    parts = version.split(".")
    if len(parts) != 3:
        raise ValueError(f"Invalid semantic version: {version}")
    return tuple(int(p) for p in parts)


def get_current_version(repo_root):
    version_file = repo_root / "version.txt"
    if not version_file.exists():
        raise RuntimeError("version.txt not found at repository root.")
    return version_file.read_text(encoding="utf-8").strip()


def list_remote_versions(repo_root):
    out = git(["ls-remote", "--tags", "--refs", "upstream", "v*"], repo_root).stdout
    versions = []
    for line in out.splitlines():
        parts = line.strip().split()
        if len(parts) != 2:
            continue
        ref = parts[1]
        if not ref.startswith("refs/tags/v"):
            continue
        v = ref.removeprefix("refs/tags/v")
        try:
            parse_semver(v)
        except ValueError:
            continue
        versions.append(v)
    versions = sorted(set(versions), key=parse_semver)
    return versions


def fetch_tag(repo_root, version):
    tag = f"v{version}"
    git(["fetch", "upstream", "tag", tag, "--force"], repo_root)
    return tag


def fetch_branch(repo_root, branch):
    git(["fetch", "upstream", branch, "--force"], repo_root)
    return f"upstream/{branch}"


def build_release_chain(local_version, target_version, available_versions):
    lv = parse_semver(local_version)
    tv = parse_semver(target_version)
    if tv < lv:
        raise RuntimeError(f"Target version {target_version} is older than local version {local_version}.")
    return [v for v in available_versions if lv < parse_semver(v) <= tv]


def build_restore_set(manifest):
    managed = manifest.get("managed_files", [])
    protected = manifest.get("protected_files", [])
    if isinstance(protected, dict):
        protected = protected.get("all_tracks", [])
    return dedupe_keep_order([*managed, *protected])


def parse_status_entries(status_text):
    entries = []
    for line in status_text.splitlines():
        if not line:
            continue
        status = line[:2]
        entry = line[3:]
        if " -> " in entry:
            entry = entry.split(" -> ", 1)[1]
        entries.append((status, entry.strip()))
    return entries


def is_ignorable_dirty_path(path):
    if path == "upgrade_start_kit.sh":
        return True
    if path == "upgrade_start_kit.py":
        return True
    name = pathlib.Path(path).name
    if name.startswith(".upgrade_start_kit.") and name.endswith(".py"):
        return True
    return False


def ensure_clean_or_warn(repo_root, allow_dirty, apply_mode):
    status_text = git(["status", "--porcelain"], repo_root).stdout
    if not status_text.strip():
        return

    status_entries = parse_status_entries(status_text)
    relevant_dirty_paths = [
        path
        for status, path in status_entries
        if status != "??" and not is_ignorable_dirty_path(path)
    ]

    if not relevant_dirty_paths:
        if not apply_mode:
            eprint("Warning: ignoring bootstrap script changes during dry-run.")
        return

    if apply_mode and not allow_dirty:
        raise RuntimeError(
            "Working tree is not clean. Blocking paths: "
            + ", ".join(relevant_dirty_paths)
            + ". Commit/stash changes or use --allow-dirty to proceed."
        )
    if not apply_mode:
        eprint(
            "Warning: working tree is not clean. Relevant dirty paths: "
            + ", ".join(relevant_dirty_paths)
            + ". Dry-run continues."
        )


def create_backup_branch(repo_root):
    ts = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    branch = f"upgrade-backup-{ts}"
    git(["branch", branch], repo_root)
    return branch


def get_ref_file_content(repo_root, ref, rel_path):
    proc = git(["show", f"{ref}:{rel_path}"], repo_root, check=False)
    if proc.returncode != 0:
        return None
    return proc.stdout


def get_ref_version(repo_root, ref):
    content = get_ref_file_content(repo_root, ref, "version.txt")
    if content is None:
        raise RuntimeError(f"Cannot read version.txt from {ref}.")
    return content.strip()


def find_changed_participant_files(manifest, repo_root, prev_ref, curr_ref):
    declared = manifest.get("participant_modifiable_changed", [])
    if declared:
        return dedupe_keep_order(declared)

    tracked = manifest.get("participant_modifiable_files", [])
    if not tracked:
        return []

    proc = git(["diff", "--name-only", prev_ref, curr_ref, "--", *tracked], repo_root, check=False)
    if proc.returncode != 0:
        return []
    return [p.strip() for p in proc.stdout.splitlines() if p.strip()]


def merge_participant_file(repo_root, rel_path, prev_ref, curr_ref):
    target = repo_root / rel_path
    ours = target.read_text(encoding="utf-8") if target.exists() else ""
    base = get_ref_file_content(repo_root, prev_ref, rel_path) or ""
    theirs = get_ref_file_content(repo_root, curr_ref, rel_path)

    if theirs is None:
        return "missing-upstream", ""

    target.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="upgrade-merge-") as td:
        td_path = pathlib.Path(td)
        ours_file = td_path / "ours"
        base_file = td_path / "base"
        theirs_file = td_path / "theirs"

        ours_file.write_text(ours, encoding="utf-8")
        base_file.write_text(base, encoding="utf-8")
        theirs_file.write_text(theirs, encoding="utf-8")

        proc = run(
            [
                "git",
                "merge-file",
                "-L",
                f"LOCAL:{rel_path}",
                "-L",
                f"BASE:{prev_ref}:{rel_path}",
                "-L",
                f"REMOTE:{curr_ref}:{rel_path}",
                str(ours_file),
                str(base_file),
                str(theirs_file),
            ],
            cwd=repo_root,
            check=False,
        )

        merged = ours_file.read_text(encoding="utf-8")
        target.write_text(merged, encoding="utf-8")

        # git merge-file exits with the number of conflict hunks (0 means clean merge).
        # So any positive code indicates conflicts, not a hard tool failure.
        if proc.returncode > 0:
            return "conflict", ""
        return "merged", ""


def apply_restore(repo_root, source_ref, paths):
    restored = []
    missing = []
    failed = []
    for p in paths:
        proc = git(
            ["restore", "--staged", "--worktree", f"--source={source_ref}", "--", p],
            repo_root,
            check=False,
        )
        if proc.returncode == 0:
            restored.append(p)
        else:
            err = (proc.stderr or "").strip()
            if "did not match any file(s) known to git" in err:
                missing.append(p)
            else:
                failed.append((p, err))
    return restored, missing, failed


def apply_deletions(repo_root, paths):
    if not paths:
        return
    git(["rm", "-r", "-f", "--ignore-unmatch", "--", *paths], repo_root)


def main():
    parser = argparse.ArgumentParser(
        description="Unified Start-Kit upgrader (manifest-driven, conflict-aware)."
    )
    parser.add_argument("--to-version", default="", help="Target Start-Kit version, e.g. 3.1.0. Default is latest available release.")
    parser.add_argument("--source-branch", default="", help="Use upstream branch (e.g. dev) as upgrade source instead of release tags.")
    parser.add_argument("--apply", action="store_true", help="Apply the upgrade. Without this flag, runs in dry-run mode.")
    parser.add_argument("--allow-dirty", action="store_true", help="Allow apply mode on a dirty working tree.")
    parser.add_argument("--remote-url", default=REMOTE_URL, help="Start-Kit remote URL.")
    args = parser.parse_args()

    try:
        repo_root = get_repo_root()
        ensure_clean_or_warn(repo_root, args.allow_dirty, args.apply)

        ensure_upstream(repo_root, args.remote_url)
        local_version = get_current_version(repo_root)
        manifests = []

        if args.source_branch:
            branch_ref = fetch_branch(repo_root, args.source_branch)
            branch_version = get_ref_version(repo_root, branch_ref)
            target_version = args.to_version or branch_version
            fetch_tag(repo_root, local_version)

            manifest, manifest_source = load_manifest(
                repo_root,
                target_version,
                raw_ref=args.source_branch,
                local_first=False,
            )
            previous_ref = f"v{local_version}"
            restore_paths = build_restore_set(manifest)
            participant_files = find_changed_participant_files(
                manifest, repo_root, previous_ref, branch_ref
            )

            manifests.append(
                {
                    "version": target_version,
                    "source": manifest_source,
                    "current_ref": branch_ref,
                    "previous_ref": previous_ref,
                    "restore_paths": restore_paths,
                    "participant_changed": participant_files,
                    "deletions": manifest.get("deleted_files", []),
                    "notes": manifest.get("migration_notes", []),
                }
            )

            print("Start-Kit upgrade plan")
            print(f"- repo: {repo_root}")
            print(f"- mode: branch")
            print(f"- source branch: {args.source_branch}")
            print(f"- source ref: {branch_ref}")
            print(f"- local version: {local_version}")
            print(f"- branch version.txt: {branch_version}")
            print(f"- manifest target version: {target_version}")
            print("- releases to apply: 1")
        else:
            remote_versions = list_remote_versions(repo_root)
            if not remote_versions:
                raise RuntimeError("No remote Start-Kit release tags found.")

            target_version = args.to_version or remote_versions[-1]
            if target_version not in remote_versions:
                raise RuntimeError(
                    f"Target version {target_version} not found in remote tags. Available latest: {remote_versions[-1]}"
                )

            chain = build_release_chain(local_version, target_version, remote_versions)

            print("Start-Kit upgrade plan")
            print(f"- repo: {repo_root}")
            print(f"- mode: release")
            print(f"- local version: {local_version}")
            print(f"- target version: {target_version}")
            print(f"- releases to apply: {len(chain)}")

            if not chain:
                print("Already up to date. Nothing to do.")
                return 0

            prev_version = local_version
            for version in chain:
                manifest, manifest_source = load_manifest(repo_root, version)
                fetch_tag(repo_root, version)
                fetch_tag(repo_root, prev_version)

                current_ref = f"v{version}"
                previous_ref = f"v{prev_version}"
                restore_paths = build_restore_set(manifest)
                participant_files = find_changed_participant_files(manifest, repo_root, previous_ref, current_ref)
                deletions = manifest.get("deleted_files", [])

                manifests.append(
                    {
                        "version": version,
                        "source": manifest_source,
                        "current_ref": current_ref,
                        "previous_ref": previous_ref,
                        "restore_paths": restore_paths,
                        "participant_changed": participant_files,
                        "deletions": deletions,
                        "notes": manifest.get("migration_notes", []),
                    }
                )
                prev_version = version

        for item in manifests:
            print(f"\nRelease {item['version']} ({item['source']})")
            print(f"- restore paths: {len(item['restore_paths'])}")
            print(f"- participant files to merge: {len(item['participant_changed'])}")
            print(f"- delete paths: {len(item['deletions'])}")
            if item["notes"]:
                print("- migration notes:")
                for n in item["notes"]:
                    print(f"  - {n}")

        if not args.apply:
            print("\nDry-run only. Re-run with --apply to execute.")
            return 0

        backup_branch = create_backup_branch(repo_root)
        print(f"\nCreated backup branch: {backup_branch}")

        total_restored = 0
        total_missing = 0
        total_failed = 0
        conflict_files = []

        for item in manifests:
            print(f"\nApplying release {item['version']}...")
            restored, missing, failed = apply_restore(repo_root, item["current_ref"], item["restore_paths"])
            apply_deletions(repo_root, item["deletions"])

            total_restored += len(restored)
            total_missing += len(missing)
            total_failed += len(failed)

            if missing:
                print("- missing restore paths in release source:")
                for p in missing:
                    print(f"  - {p}")

            if failed:
                print("- failed restore paths:")
                for p, err in failed:
                    print(f"  - {p}: {err}")
                return 2

            merge_errors = []
            for rel_path in item["participant_changed"]:
                status, detail = merge_participant_file(
                    repo_root, rel_path, item["previous_ref"], item["current_ref"]
                )
                if status == "conflict":
                    conflict_files.append(rel_path)
                elif status == "error":
                    merge_errors.append((rel_path, detail))

            if merge_errors:
                print("- merge errors in participant-modifiable files:")
                for p, err in merge_errors:
                    print(f"  - {p}: {err}")
                return 3

        print("\nApply summary")
        print(f"- restored: {total_restored}")
        print(f"- missing in release source: {total_missing}")
        print(f"- failed restore: {total_failed}")
        print(f"- participant file conflicts: {len(conflict_files)}")

        if conflict_files:
            print("\nResolve these files like normal git merge conflicts:")
            for p in dedupe_keep_order(conflict_files):
                print(f"- {p}")

        status = git(["status", "--short"], repo_root).stdout
        print("\nCurrent git status:")
        print(status if status.strip() else "(clean)")
        print("Upgrade applied. Review, test, then commit.")
        return 0

    except RuntimeError as ex:
        eprint(f"Error: {ex}")
        return 1
    except subprocess.CalledProcessError as ex:
        eprint("Command failed:")
        eprint(" ".join(ex.cmd))
        eprint(ex.stderr or "")
        return ex.returncode or 1


if __name__ == "__main__":
    raise SystemExit(main())
