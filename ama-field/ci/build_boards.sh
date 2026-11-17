#!/usr/bin/env bash
# Build ArduPlane for each board given on the command line and collect the
# release files. A board that fails to configure, compile or link is recorded
# as failed; the script itself only fails on bad usage.
#
# usage: ama-field/ci/build_boards.sh <release-version> <board> [<board> ...]
#   e.g. ama-field/ci/build_boards.sh v17 FlyingRCF4WingMini MatekF405-Wing
#
# Output (OUT_DIR, default ama-field-out):
#   files/ArduPlane-<fw>_AMAFieldMode_<version>_<board>.apj / _with_bl.hex
#   results/<board>.tsv   board, status, flash used, flash free, files, reason
#   logs/<board>.log      full configure + build log

set -u

if [ $# -lt 2 ]; then
    echo "usage: $0 <release-version> <board> [<board> ...]" >&2
    exit 2
fi

VERSION=$1
shift
OUT=${OUT_DIR:-ama-field-out}
mkdir -p "$OUT/files" "$OUT/results" "$OUT/logs"

# "ArduPlane V4.7.2" -> 4.7.2
FW=$(sed -n 's/^#define THISFIRMWARE "ArduPlane V\([^" ]*\).*/\1/p' ArduPlane/version.h)
if [ -z "$FW" ]; then
    echo "could not read the firmware version from ArduPlane/version.h" >&2
    exit 2
fi
PREFIX="ArduPlane-${FW}_AMAFieldMode_${VERSION}"

export NOCOLOR=1

for B in "$@"; do
    LOG="$OUT/logs/$B.log"
    STATUS=failed
    USED=
    FREE=
    FILES=
    REASON=

    echo "::group::$B"
    START=$(date +%s)
    if ./waf configure --board "$B" >"$LOG" 2>&1 && ./waf plane >>"$LOG" 2>&1; then
        BIN="build/$B/bin"
        if [ -f "$BIN/arduplane.apj" ]; then
            STATUS=ok
            cp "$BIN/arduplane.apj" "$OUT/files/${PREFIX}_${B}.apj"
            FILES="${PREFIX}_${B}.apj"
            if [ -f "$BIN/arduplane_with_bl.hex" ]; then
                cp "$BIN/arduplane_with_bl.hex" "$OUT/files/${PREFIX}_${B}_with_bl.hex"
                FILES="$FILES ${PREFIX}_${B}_with_bl.hex"
            fi
            # waf size table row: bin/arduplane text data bss total free ...
            ROW=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG" | awk '$1 == "bin/arduplane" { r = $0 } END { print r }')
            USED=$(echo "$ROW" | awk '{ print $5 }')
            FREE=$(echo "$ROW" | awk '{ print $6 }')
            case "$USED" in *[!0-9]*) USED= ;; esac
            case "$FREE" in *[!0-9]*) FREE= ;; esac
        else
            REASON="build finished but no arduplane.apj was produced"
        fi
    else
        OVER=$(grep -o "region [\`']*[A-Za-z0-9_]*'* overflowed by [0-9]* bytes" "$LOG" | head -n 1)
        if [ -n "$OVER" ]; then
            REASON="does not fit: $OVER"
        elif grep -q "is not a valid board\|unknown board\|Board .* not found" "$LOG"; then
            REASON="unknown board"
        else
            REASON="build error, see $B.log in the workflow artifacts"
        fi
    fi
    END=$(date +%s)
    tail -n 25 "$LOG"
    echo "::endgroup::"

    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$B" "$STATUS" "$USED" "$FREE" "$FILES" "$REASON" >"$OUT/results/$B.tsv"
    if [ "$STATUS" = ok ]; then
        echo "$B: OK, flash used ${USED:-?} B, free ${FREE:-?} B ($((END - START)) s)"
    else
        echo "::warning title=$B not built::$REASON"
        echo "$B: FAILED, $REASON ($((END - START)) s)"
    fi

    # keep runner disk use down; the files we need are already copied
    rm -rf "build/$B"
done

exit 0
