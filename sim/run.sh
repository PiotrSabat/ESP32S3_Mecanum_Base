#!/usr/bin/env bash
# Builds and runs every simulator on the host machine (not on the ESP32).
# Usage:  ./sim/run.sh          - all of them
#         ./sim/run.sh failsafe - just one
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$ROOT/sim"
OUT="$SIM/build"
mkdir -p "$OUT"

# Motor.cpp is needed everywhere; MecanumDrive.cpp only by the kinematics
# test, but linking it in every time breaks nothing.
SRC="$ROOT/src/Motor.cpp $ROOT/src/MecanumDrive.cpp"
INC="-I$SIM/stubs -I$ROOT/src"

TESTS=${1:-"failsafe kinematics braking holding"}
FAILED=0

for t in $TESTS; do
    printf '\n=========== %s ===========\n' "$t"
    if ! g++ -std=c++17 -Wall -Wextra $INC -o "$OUT/$t" "$SIM/$t.cpp" $SRC; then
        echo "COMPILATION FAILED: $t"
        FAILED=1
        continue
    fi
    "$OUT/$t" || FAILED=1
done

printf '\n'
if [ "$FAILED" -eq 0 ]; then
    echo "=== All simulations finished without failures ==="
else
    echo "=== THERE ARE FAILURES - see above ==="
fi
exit $FAILED
