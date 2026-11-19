#!/usr/bin/env bash
# Build ArduPlane for each board given on the command line and collect the
# release files. A board that fails to configure, compile or link is recorded
# as failed; the script itself only fails on bad usage.
#
# Every build gets field-mode.hwdef (baro thrust and wind compensation). Per
# board, stopping at the first that fits:
#   1. stock hwdef + field-mode.hwdef
#   2. trim-tier1.hwdef + field-mode.hwdef   } only a 1 MB board that is not
#   3. trim-tier2.hwdef + field-mode.hwdef   } listed in no_trim.txt
#   4. trim-tier2.hwdef alone: no baro compensation (the learned
#      AUTOLAND_TD_OFS takes up the larger float; a board that can't be trimmed
#      tries its stock hwdef alone here)
#
# usage: ama-field/ci/build_boards.sh <release-version> <board> [<board> ...]
#   e.g. ama-field/ci/build_boards.sh v17 FlyingRCF4WingMini MatekF405-Wing
#
# Output (OUT_DIR, default ama-field-out):
#   files/ArduPlane-<fw>_AMAFieldMode_<version>_<board>.apj / _with_bl.hex
#   results/<board>.tsv          board, status, build, flash used, flash free, files, reason, baro comp
#   results/<board>.removed.tsv  features a trimmed build removed (define, description)
#   features/<board>.txt         Tools/scripts/extract_features.py output for the shipped build
#   logs/<board>[-<step>].log    full configure + build log of each attempt

set -u

if [ $# -lt 2 ]; then
    echo "usage: $0 <release-version> <board> [<board> ...]" >&2
    exit 2
fi

VERSION=$1
shift
OUT=${OUT_DIR:-ama-field-out}
CI=ama-field/ci
mkdir -p "$OUT/files" "$OUT/results" "$OUT/logs" "$OUT/features" "$OUT/probe" "$OUT/overlays"

# "ArduPlane V4.7.2" -> 4.7.2
FW=$(sed -n 's/^#define THISFIRMWARE "ArduPlane V\([^" ]*\).*/\1/p' ArduPlane/version.h)
if [ -z "$FW" ]; then
    echo "could not read the firmware version from ArduPlane/version.h" >&2
    exit 2
fi
PREFIX="ArduPlane-${FW}_AMAFieldMode_${VERSION}"

# without intelhex, waf silently skips the _with_bl.hex files
if ! python3 -c "import intelhex" 2>/dev/null; then
    echo "python3 intelhex module missing: install it (pip3 install intelhex) so _with_bl.hex files are built" >&2
    exit 2
fi
NO_TRIM=$(python3 -c "import sys; sys.path.insert(0, '$CI'); import release_tools as r; print(' '.join(r.read_list('$CI/no_trim.txt')))")

# a trim tier plus the field-mode overlay (--extra-hwdef takes one file)
FIELD="$PWD/$CI/field-mode.hwdef"
for N in 1 2; do
    printf 'include %s\ninclude %s\n' "$PWD/$CI/trim-tier$N.hwdef" "$FIELD" > "$OUT/overlays/field-tier$N.hwdef"
done

export NOCOLOR=1

# build_variant <board> <log> [extra-hwdef]; returns waf's status
build_variant() {
    local b=$1 log=$2 extra=${3:-}
    if [ -n "$extra" ]; then
        ./waf configure --board "$b" --extra-hwdef="$extra" >"$log" 2>&1 && ./waf plane >>"$log" 2>&1
    else
        ./waf configure --board "$b" >"$log" 2>&1 && ./waf plane >>"$log" 2>&1
    fi
}

# bytes over the flash limit from a failed link, or empty
overflow_bytes() {
    local n m
    n=$(grep -o "overflowed by [0-9]* bytes" "$1" | head -n 1 | grep -o "[0-9][0-9]*")
    if [ -z "$n" ]; then
        # ChibiOS common.ld ends flash with the .crash_log section, so code past the end of
        # flash fails as "cannot move location counter backwards (from <code end> to <flash end>)"
        m=$(grep -o "cannot move location counter backwards (from [0-9a-fA-Fx]* to [0-9a-fA-Fx]*)" "$1" | head -n 1)
        if [ -n "$m" ]; then
            set -- $(echo "$m" | sed 's/.*from \(0x\)\{0,1\}\([0-9a-fA-F]*\) to \(0x\)\{0,1\}\([0-9a-fA-F]*\).*/\2 \4/')
            n=$(( 16#$1 - 16#$2 ))
        fi
    fi
    echo "$n"
}

# yes / no / ? from extract_features.py output: both baro compensations compiled in
baro_comp() {
    local f=$1
    if ! grep -q "_ENABLED" "$f" 2>/dev/null; then
        echo "?"
    elif grep -qx "HAL_BARO_WIND_COMP_ENABLED" "$f" && grep -qx "AP_BARO_THST_COMP_ENABLED" "$f"; then
        echo "yes"
    else
        echo "no"
    fi
}

for B in "$@"; do
    STATUS=failed
    TIER=stock
    USED=
    FREE=
    FILES=
    REASON=
    BARO=
    LOG="$OUT/logs/$B.log"
    rm -f "$OUT/results/$B.removed.tsv"

    echo "::group::$B"
    START=$(date +%s)
    if build_variant "$B" "$LOG" "$FIELD"; then
        STATUS=ok
    else
        OVER=$(overflow_bytes "$LOG")
        FLASH_KB=$(sed -n 's/^#define BOARD_FLASH_SIZE \([0-9]*\).*/\1/p' "build/$B/hwdef.h" 2>/dev/null)
        TRIMMABLE=1
        NOT_TRIMMED=
        if [ "${FLASH_KB:-0}" -gt 1024 ] || [ "${FLASH_KB:-0}" -eq 0 ]; then
            TRIMMABLE=0
            NOT_TRIMMED="more than 1 MB flash"
        elif [[ " $NO_TRIM " == *" $B "* ]]; then
            TRIMMABLE=0
            NOT_TRIMMED="listed in no_trim.txt"
        fi
        if [ -z "$OVER" ]; then
            if grep -q "is not a valid board\|unknown board\|Board .* not found" "$LOG"; then
                REASON="unknown board"
            else
                REASON="build error, see $B.log in the workflow artifacts"
            fi
        elif [ $TRIMMABLE -eq 0 ]; then
            # last resort: the board's own hwdef without the field-mode overlay
            echo "$B: $OVER bytes too big, not trimmed ($NOT_TRIMMED); trying without baro compensation"
            NLOG="$OUT/logs/$B-nobaro.log"
            if build_variant "$B" "$NLOG"; then
                STATUS=ok
                LOG="$NLOG"
            else
                NOVER=$(overflow_bytes "$NLOG")
                REASON="does not fit: ${OVER} bytes over, ${NOVER:-?} without baro compensation (not trimmed: $NOT_TRIMMED)"
            fi
        else
            echo "$B: stock build is $OVER bytes too big, trying trim tiers"
            if ! python3 $CI/release_tools.py probe --build-dir "build/$B" --out "$OUT/probe/$B.stock.json"; then
                REASON="does not fit (${OVER} bytes over) and the feature probe failed"
            else
                # step: name, overlay, trimdiff option
                for STEP in "tier1|$OUT/overlays/field-tier1.hwdef|" \
                            "tier2|$OUT/overlays/field-tier2.hwdef|" \
                            "tier2-nobaro|$PWD/$CI/trim-tier2.hwdef|--no-baro-comp"; do
                    IFS='|' read -r NAME OVL OPT <<< "$STEP"
                    TLOG="$OUT/logs/$B-$NAME.log"
                    if build_variant "$B" "$TLOG" "$OVL"; then
                        python3 $CI/release_tools.py probe --build-dir "build/$B" --out "$OUT/probe/$B.$NAME.json" &&
                        python3 $CI/release_tools.py trimdiff $OPT --stock "$OUT/probe/$B.stock.json" \
                            --trimmed "$OUT/probe/$B.$NAME.json" --out "$OUT/results/$B.removed.tsv" \
                            > "$OUT/probe/$B.$NAME.diff" 2>&1
                        PROBE_RC=$?
                        cat "$OUT/probe/$B.$NAME.diff" 2>/dev/null
                        if [ $PROBE_RC -eq 0 ]; then
                            STATUS=ok
                            TIER="tier ${NAME:4:1}"
                            LOG="$TLOG"
                            REASON=
                        else
                            REASON="$NAME fits but the trim check failed: $(tail -n 1 "$OUT/probe/$B.$NAME.diff" 2>/dev/null)"
                            rm -f "$OUT/results/$B.removed.tsv"
                        fi
                        break
                    fi
                    TOVER=$(overflow_bytes "$TLOG")
                    if [ -z "$TOVER" ]; then
                        REASON="$NAME build error, see $B-$NAME.log in the workflow artifacts"
                        break
                    fi
                    echo "$B: $NAME is still $TOVER bytes too big"
                    REASON="does not fit even with tier 2 and no baro compensation: ${TOVER} bytes over (stock: ${OVER} bytes over)"
                done
            fi
        fi
    fi

    if [ "$STATUS" = ok ]; then
        BIN="build/$B/bin"
        if [ -f "$BIN/arduplane.apj" ]; then
            cp "$BIN/arduplane.apj" "$OUT/files/${PREFIX}_${B}.apj"
            FILES="${PREFIX}_${B}.apj"
            if [ -f "$BIN/arduplane_with_bl.hex" ]; then
                cp "$BIN/arduplane_with_bl.hex" "$OUT/files/${PREFIX}_${B}_with_bl.hex"
                FILES="$FILES ${PREFIX}_${B}_with_bl.hex"
            fi
            # same numbers Mission Planner shows: image size and free flash from the .apj
            read -r USED FREE < <(python3 -c "import json, sys; d = json.load(open(sys.argv[1])); print(d['image_size'], d['flash_free'])" "$BIN/arduplane.apj")
            python3 Tools/scripts/extract_features.py "$BIN/arduplane" > "$OUT/features/$B.txt" 2>&1 || true
            BARO=$(baro_comp "$OUT/features/$B.txt")
        else
            STATUS=failed
            REASON="build finished but no arduplane.apj was produced"
        fi
    fi
    END=$(date +%s)
    tail -n 25 "$LOG"
    echo "::endgroup::"

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$B" "$STATUS" "$TIER" "$USED" "$FREE" "$FILES" "$REASON" "$BARO" >"$OUT/results/$B.tsv"
    if [ "$STATUS" = ok ]; then
        echo "$B: OK ($TIER, baro compensation: $BARO), flash used ${USED:-?} B, free ${FREE:-?} B ($((END - START)) s)"
    else
        echo "::warning title=$B not built::$REASON"
        echo "$B: FAILED, $REASON ($((END - START)) s)"
    fi

    # Keep runner disk use down; the files we need are already copied. All of
    # build/ goes, because waf's shared configuration cache (build/c4che) still
    # refers to the files of a deleted board directory.
    rm -rf build
done

exit 0
