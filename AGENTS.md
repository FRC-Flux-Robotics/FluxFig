# AGENTS.md — FluxFig Repository Guide (FRC Team 10413)

## 1. Repository Overview & Scope
`FluxFig` (`frc.lib.fluxfig`) is the **standalone pure-Java 17 configuration engine, binary `.wpilog` codec, and 14-metric `system-snapshot.txt` evaluator** (`--release 17`, zero WPILib or external Maven dependencies) for **FRC Team 10413 (FLUX Robotics)**:
- [`README.md`](README.md): Full API, CLI usage, CSV schemas, and two-mode (`PRODUCTION` vs. `TUNING`) workflow.
- [`src/main/java/frc/lib/fluxfig/`](src/main/java/frc/lib/fluxfig/):
  - `FluxFig.java`: Public facade, `<Profile>:<Overlay> -> <Profile> -> DEFAULT` resolution, `@RowKey` lookup, linear interpolation, live `TUNING` polling, and `promoteSnapshotToDirectory(...)`.
  - `FluxFigCsvCodec.java`: RFC-4180 CSV parser, cross-OS `ConfigSHA256`, `.wpilog` snapshot codec, and `#`-comment-preserving CSV writer.
  - `FluxFigRecordBinder.java`: Reflective Java `record` binder (`@DefaultValue`, `@RowKey`, `@Tunable`).
  - `FluxFigValidator.java`: Strict hardware safety validator (`60 A` stator/slip, `40 A` supply, SystemCore odd CAN FD bus blocking `can_s1`/`can_s3`, cross-profile CAN ID collision checks).
  - `FluxFigSheetSync.java`: Direct Google Sheets export puller with atomic `validateStrict()` pre-write gate.
  - `FluxFigMain.java`: CLI entry point (`validate`, `pull`, `promote`, `release`, `sha256`, `get`, `tables`, `snapshot`).
  - `FluxWpiLogCodec.java`: Zero-WPILib binary `.wpilog` v1.0 reader & writer.
  - `FluxSystemSnapshot.java`: 14-metric SystemCore & roboRIO `system-snapshot.txt` capture script, parser, and evaluator.

## 2. Build, Test & Output Rules
1. **Pure Java 17 Bytecode (`--release 17`, Zero External Dependencies):**
   - Compile with `javac --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror` so `dist/fluxfig.jar` runs on any JDK 17+ runtime.
   - Never import `edu.wpi.first.*`, `org.wpilib.*`, or third-party Maven libraries.
2. **Self-Contained Build Output:**
   - `scripts/build.sh` and `scripts/build.bat` must write outputs strictly inside `build/` and `dist/fluxfig.jar` (never outside this repository root).
3. **Verification Command:**
   ```bash
   ./scripts/build.sh
   ```
