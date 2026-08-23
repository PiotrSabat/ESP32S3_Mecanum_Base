#!/usr/bin/env bash
# Buduje i uruchamia wszystkie symulatory na komputerze (nie na ESP32).
# Użycie:  ./sim/run.sh          — wszystkie
#          ./sim/run.sh failsafe — jeden wybrany
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$ROOT/sim"
OUT="$SIM/build"
mkdir -p "$OUT"

# Motor.cpp jest potrzebny wszędzie; MecanumDrive.cpp tylko kinematyce,
# ale dołączenie go zawsze niczego nie psuje.
SRC="$ROOT/src/Motor.cpp $ROOT/src/MecanumDrive.cpp"
INC="-I$SIM/stubs -I$ROOT/src"

TESTS=${1:-"failsafe kinematyka hamowanie trzymanie"}
FAILED=0

for t in $TESTS; do
    printf '\n=========== %s ===========\n' "$t"
    if ! g++ -std=c++17 -w $INC -o "$OUT/$t" "$SIM/$t.cpp" $SRC; then
        echo "BLAD KOMPILACJI: $t"
        FAILED=1
        continue
    fi
    "$OUT/$t" || FAILED=1
done

printf '\n'
if [ "$FAILED" -eq 0 ]; then
    echo "=== Wszystkie symulacje zakonczone bez bledow ==="
else
    echo "=== SA BLEDY - patrz wyzej ==="
fi
exit $FAILED
