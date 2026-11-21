#!/usr/bin/env bash
# Local AMA field build, the same way the release CI builds: same Docker image
# (ardupilot/ardupilot-dev-chibios:v0.1.3), same ama-field/ci/build_boards.sh, so
# the same overlays, trim tiers and file names. Runs in WSL or on Linux; started
# by ama-field/ci/build_local.py (VS Code task "AMA field: Build locally").
#
# usage: build_local_wsl.sh <repo> <changes.tar|-> <deleted.txt|-> <version> <board> [<board> ...]
#   <repo>         the working copy (e.g. /mnt/c/.../Ardupilot_AMA_Field)
#   <changes.tar>  its uncommitted and untracked files, as git would commit them
#   <deleted.txt>  files deleted in the working copy, one per line
#
# The build runs in a Linux clone kept at $AMA_BUILD_TREE (default
# ~/ama-field-build), with the submodules and the build cache. The first run clones
# it and fetches the submodules (several minutes); later runs only update it. The
# results are copied to <repo>/ama-field-out.

set -eu

if [ $# -lt 5 ]; then
    echo "usage: $0 <repo> <changes.tar|-> <deleted.txt|-> <version> <board> [<board> ...]" >&2
    exit 2
fi
REPO=$1
CHANGES=$2
DELETED=$3
VERSION=$4
shift 4
BOARDS="$*"
TREE=${AMA_BUILD_TREE:-$HOME/ama-field-build}
IMAGE=ardupilot/ardupilot-dev-chibios:v0.1.3

if ! command -v docker > /dev/null 2>&1; then
    echo "docker not found: install Docker (Docker Engine in WSL, or Docker Desktop with WSL integration)" >&2
    exit 2
fi

if [ ! -d "$TREE/.git" ]; then
    echo "First run: cloning into $TREE and fetching the submodules (one-off, several minutes)..."
    git clone --quiet --no-checkout "$REPO" "$TREE"
fi
cd "$TREE"

# the committed state of the working copy
git fetch --quiet "$REPO" HEAD
git checkout --quiet --force --detach FETCH_HEAD
git clean -ffdq -e /build/ -e /ama-field-out/
git submodule sync --quiet --recursive
git submodule update --quiet --init --recursive

# plus its uncommitted changes
if [ "$CHANGES" != "-" ]; then
    tar -xf "$CHANGES"
fi
if [ "$DELETED" != "-" ]; then
    tr -d '\r' < "$DELETED" | while IFS= read -r f; do
        [ -n "$f" ] && rm -f -- "$f"
    done
fi

rm -rf ama-field-out
mkdir -p "$HOME/.ccache-ama"
RC=0
docker run --rm -v "$TREE:/w" -v "$HOME/.ccache-ama:/root/.ccache" -w /w \
    -e VERSION="$VERSION" -e BOARDS="$BOARDS" "$IMAGE" bash -c '
        set -e
        git config --global --add safe.directory "*"
        python3 -c "import intelhex" 2> /dev/null || python3 -m pip install --user --quiet intelhex==2.3.0
        . .github/workflows/ccache.env
        ccache -M 2G > /dev/null
        PATH="/usr/lib/ccache:/opt/gcc-arm-none-eabi-10/bin:/root/.local/bin:$PATH"
        bash ama-field/ci/build_boards.sh "$VERSION" $BOARDS
    ' || RC=$?

# the container ran as root: hand the files back before copying them out
docker run --rm -v "$TREE:/w" "$IMAGE" chown -R "$(id -u):$(id -g)" /w/ama-field-out /w/build > /dev/null 2>&1 || true
rm -rf "$REPO/ama-field-out"
cp -r ama-field-out "$REPO/"
exit $RC
