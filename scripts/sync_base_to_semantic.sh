#!/usr/bin/env bash
# ==============================================================================
# Hospobot Git Branch Sync Tool: BaseNav -> SemanticNav
# ==============================================================================
# Safely propagates all updates, bugfixes, and calibrations from the BaseNav
# branch into the downstream SemanticNav branch.
# ==============================================================================

set -e

# Change to repository root
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

echo "======================================================"
echo " Hospobot Sync: BaseNav -> SemanticNav"
echo " Workspace: $WS_ROOT"
echo "======================================================"

# Check for uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo "❌ Error: Working directory has uncommitted or staged changes."
    echo "Please commit or stash your changes before syncing branches:"
    echo "   git status"
    exit 1
fi

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "-> Fetching latest updates from origin..."
git fetch origin

echo "-> Checking out SemanticNav..."
git checkout SemanticNav

echo "-> Merging BaseNav into SemanticNav..."
git merge BaseNav -m "chore: sync BaseNav updates into SemanticNav"

echo "✅ Merge completed successfully!"

# Check if auto-push requested or prompt
if [ "$1" == "--push" ] || [ "$1" == "-p" ]; then
    echo "-> Pushing SemanticNav to origin..."
    git push origin SemanticNav
    echo "✅ SemanticNav pushed to GitHub!"
else
    echo ""
    echo "Tip: Run 'git push origin SemanticNav' to update GitHub, or run this script with --push"
fi

# Restore branch if started from another branch
if [ "$CURRENT_BRANCH" != "SemanticNav" ]; then
    echo "-> Switching back to $CURRENT_BRANCH..."
    git checkout "$CURRENT_BRANCH"
fi

echo "======================================================"
echo " Sync Complete!"
echo "======================================================"
