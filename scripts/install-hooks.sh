#!/usr/bin/env bash
# install-hooks.sh — instala los git hooks del proyecto
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "🔧 Instalando git hooks..."

# pre-commit: clang-format
cat > "$REPO_ROOT/.git/hooks/pre-commit" << 'HOOK'
#!/usr/bin/env bash
# pre-commit — auto-formats C/H files with clang-format (Allman)
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🔍 Pre-commit: clang-format (Allman)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

"$REPO_ROOT/scripts/pre-commit-clang-format.sh"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✅ Format OK. Commit permitted."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
HOOK

chmod +x "$REPO_ROOT/.git/hooks/pre-commit"

echo ""
echo "✅ Hooks instalados:"
echo "   .git/hooks/pre-commit  → clang-format (Allman) en staged .c/.h"
echo ""
echo "Para verificar: cat .git/hooks/pre-commit | head -5"
