#!/usr/bin/env bash
# pre-commit hook: auto-fix C/C++ formatting with clang-format
# Only runs if there are staged .c or .h files.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# Get staged C/H files in the project
STAGED_FILES=$(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(c|h)$' || true)

if [ -z "$STAGED_FILES" ]; then
  echo "  → No staged C/H files — skipping format check."
  exit 0
fi

if ! command -v clang-format &>/dev/null; then
  echo "  ⚠ clang-format not found — skipping format check."
  echo "    Install it with: sudo apt install -y clang-format"
  exit 0
fi

echo "  → Staged C/H files:"
echo "$STAGED_FILES" | sed 's/^/     /'

echo "  → Auto-fixing formatting with clang-format (Allman)..."
echo "$STAGED_FILES" | while IFS= read -r f; do
    if [ -f "$f" ]; then
        clang-format --style=file -i "$f"
    fi
done

# Re-stage any files that were modified by clang-format
MODIFIED=$(git diff --name-only --diff-filter=M | grep -E '\.(c|h)$' || true)
if [ -n "$MODIFIED" ]; then
  echo "  → Re-staging formatted file(s)..."
  echo "$MODIFIED" | xargs git add
fi

echo "  → Formatting OK"
