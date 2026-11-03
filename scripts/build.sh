#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

rm -rf build/classes/main build/classes/test
mkdir -p build/classes/main build/classes/test build/libs dist

echo "[1/3] Compiling FluxFig (Java 17 bytecode, zero external dependencies)..."
javac --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror -d build/classes/main $(find src/main/java -name "*.java" | sort)
javac --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror -cp build/classes/main -d build/classes/test $(find src/test/java -name "*.java" | sort)

echo "[2/3] Running FluxFig unit, safety & Google Sheet sync tests..."
java -ea -cp build/classes/main:build/classes/test frc.lib.fluxfig.FluxFigTest

echo "[3/3] Packaging build/libs/fluxfig-1.0.0.jar & dist/fluxfig.jar..."
jar --create --date=1980-01-01T00:00:02Z --file build/libs/fluxfig-1.0.0.jar --main-class frc.lib.fluxfig.FluxFigMain -C build/classes/main .
cp build/libs/fluxfig-1.0.0.jar dist/fluxfig.jar

echo "Done! Library & CLI JAR: $ROOT_DIR/dist/fluxfig.jar"
