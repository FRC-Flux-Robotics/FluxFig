# FluxFig — Standalone CSV & Google Sheets Configuration Engine

Pure-Java 17 configuration engine (`--release 17`, `zero` WPILib or external dependencies) for **FRC Team 10413 (FLUX Robotics)** and any FRC team using Google Sheets and Git-tracked CSV configuration tables (`parameters.csv`, `mechanisms.csv`, `cameras.csv`, `batteries.csv`, `releases.csv`, or custom season tables like `elevator_presets.csv` or `range_table.csv`).

Because `FluxFig` compiles to Java 17 bytecode with zero external dependencies, a single compiled `dist/fluxfig.jar` (`fluxfig-1.0.0.jar`) runs unchanged on both Java 17 (roboRIO) and Java 25 (SystemCore & laptops) across:
- **`Flux-SystemCore-2027`** (WPILib 2027 / JDK 25)
- **`Flux-RoboRIO-2027` & `New_Robot`** (`BackTo2026` roboRIO fallback / JDK 17)
- **`FluxTray.jar`** (Driver Station SFTP / mode-switcher / version harvester / Java 17+ bytecode, runs on JDK 17 & JDK 25)
- **`FluxProcessor`** (Post-match telemetry & battery analyzer / Java 17+ bytecode, runs on JDK 17 & JDK 25)

---

## Key Capabilities
1. **Production Mode (`Mode.PRODUCTION`) vs. Tuning Mode (`Mode.TUNING`) & End-of-Session Version Harvesting:**
   - **Production Mode (Default & Auto-Locked on FMS via `lockProductionOnFms(true)`):** Tables and parameters are locked to validated CSV constants (`0 ns` NetworkTables overhead; dashboard widgets cannot alter robot behavior).
   - **Tuning Mode (`enableTuning(TuningBackend)` or `mode=TUNING` in `active-profile.txt`):** Automatically publishes `@FluxFig.Tunable` record columns (e.g., `/FluxFig/Tuning/range_table/MID/shooterRps`, `/FluxFig/Tuning/range_table/MID/hoodRot`) and operational parameters (`/FluxFig/Tuning/parameters/IntakeInSpeed`) to the dashboard while excluding structural hardware constants (`controllerProfile`, `*CanBus`, `*LimitAmps`).
   - **Live Poll & Temporary RAM Snapshot (`pollTuning()`):** Polls changed dashboard values, updates in-memory parameter and relational table cells on the fly, and writes `/tmp/fluxfig-tuned-snapshot.txt` (RAM disk—wiped on reboot, never mutating `/home/lvuser/deploy/configs/*.csv` behind Git's back).
   - **End-of-Session Harvest & Version Promotion (`promoteSnapshotToDirectory` / CLI `promote`):** `FluxTray` (or CLI) pulls `/tmp/fluxfig-tuned-snapshot.txt`, applies all tuned diffs to the laptop's Git-tracked `.csv` files (preserving `#` comments and custom columns), validates via `validateStrict()`, and automatically appends the next `vN` release row to `releases.csv` with an auto-generated diff summary.
2. **Unified Preset Lookup & Continuous Distance Interpolation (`getRowByKey`, `interpolateTable`):**
   - A single relational table (such as `range_table.csv` or `elevator_presets.csv`) powers both **named button presets** (`fig.getRequiredRowByKey("range_table", ShotPreset.MID, ShotRow.class)`) and **continuous distance interpolation** (`fig.interpolateTable("range_table", "distanceMeters", currentDistance, ShotRow.class)`), eliminating split-brain between preset tuning and auto-ranging.
3. **Direct Google Sheets & Local CSV Loading (`pullFromGoogleSheet`, `loadFromGoogleSheet`, `loadWithSheetSync`, `load`):**
   - Pulls tabs directly from Google Sheets (`https://docs.google.com/spreadsheets/d/<SheetID>/gviz/tq?tqx=out:csv&sheet=<Tab>`, when shared as *"Anyone with the link can view"*) or from `fluxfig-sheet.properties` (`sheetId=...`, `tabs=...`) using JDK 17's built-in `HttpClient`.
   - **Atomic Safety Gate Before Overwrite:** Validates downloaded sheet tabs in memory with `validateStrict()` first; if any sheet cell violates a safety rule (`> 60 A` stator/slip, `> 40 A` supply, `can_s1`/`can_s3`, duplicate CAN IDs, invalid `gravityType`), the pull is rejected and local offline CSVs remain untouched.
   - **Offline Pit Fallback (`loadWithSheetSync`):** Syncs from Google Sheets when internet is available and falls back seamlessly to local `configDir/*.csv` files in the competition pit.
4. **1-Cell SystemCore $\leftrightarrow$ roboRIO Fallback & Hierarchical Profile Inheritance (`<Profile>:<Overlay> -> <Profile> -> DEFAULT`):**
   - Switch `COMP_2027` between SystemCore (`SYSTEMCORE_2027`) and a roboRIO 2.0 + CANivore fallback (`ROBORIO_CANIVORE_2026`) by changing a single cell or activating `:roborio_fallback`.
   - Automatically remaps `MechanismSpec.canBus()` across profile overlays (`can_s0` $\rightarrow$ `swerveCanBus`/`driveCanBus` or `"Drivetrain"`, `can_s2` $\rightarrow$ `mechCanBus` or `"Mech"`, `can_s4` $\rightarrow$ `auxCanBus` or `"rio"`, or symbolic `$swerveCanBus`/`$mechCanBus` references) without duplicating rows in `mechanisms.csv`.
   - Applies across `getString`/`getDouble`/`getInt`/`getBoolean`, `getMechanismSpec([profile,] mechanism)`, `getCamerasForRobot(profile)` / `getCamerasForActiveProfile()`, and `getTable(tableName, profile, Record.class)` (merging rows by `@RowKey` so profile-specific rows cleanly override `DEFAULT` rows without duplicates).
5. **Cross-OS Deterministic `ConfigSHA256`, `.wpilog` Snapshot Re-Hydration & Comment-Preserving CSV Write-Back:**
   - Normalizes `\r\n`, `\n`, and UTF-8 BOM (`\uFEFF`) so Windows Driver Station laptops and Linux/SystemCore compute the exact same 8-character `ConfigSHA256` digest.
   - Generates a deterministic full snapshot (`getEffectiveSnapshot()`) for `.wpilog` metadata (`/FluxFig/EffectiveSnapshot`) and reconstructs the exact in-memory `FluxFig` instance (including `Mode`, `ActiveProfile`, `RuntimeOverrides`, and `TableCellOverrides`) via `FluxFig.fromEffectiveSnapshot(snapshot)`.
   - Preserves leading `#` comments, `@Profile` header prefixes, and custom columns (`notes`) across `saveCameraSpecs`, `saveMechanismSpecs`, `saveBatterySpecs`, `persistOverridesToCsv`, and `promoteSnapshotToDirectory`.
6. **Hardware Safety Validator (`validateStrict()`):**
   - Enforces `(0, 60.0] A` stator/slip current limits and `(0, 40.0] A` supply limits across `mechanisms.csv`, `parameters.csv`, and live overrides.
   - Blocks odd-numbered SystemCore SPI-shared CAN FD ports (`can_s1`, `can_s3`) across raw and profile-resolved CAN buses.
   - Validates `gravityType` (`NONE`, `ELEVATOR_STATIC`, `ARM_COSINE`), CAN IDs in `[0, 62]`, positive `gearRatio`, mechanism soft limits (`minPosRot <= maxPosRot`), row/column numeric & enum parse integrity (`tableName [row N, col '...']`), duplicate `key` rows across Key-Value Matrix files, duplicate `(robot, mechanism)` rows, duplicate `(robot, canBus, motorId)` assignments across both raw and profile-remapped CAN buses, duplicate `(robot, cameraName)` / `(robot, portIndex)` assignments, SystemCore USB camera `portIndex in [0, 3]` bounds, finite 6-DOF camera transforms, and battery $R_{\text{int}}$ thresholds.

---

## Built-In CSV & Google Sheets Tab Schemas

| Tab / File | Tab Type | Columns & Units |
| :--- | :--- | :--- |
| **`parameters.csv`** | Key-Value Matrix (`Key` in col 0) | `key,DEFAULT,<Profile>,<Profile>:<Overlay>,...` (Canonical fleet keys: `controllerProfile`, `loopPeriodMs`, `swerveCanBus`, `mechCanBus`, `auxCanBus`, `visionBackend`, `coordinateSystem`, `simulateMissingMechanisms`, `maxLinearSpeedMps`, `maxAngularSpeedRadPerSec`, `driveStatorLimitAmps`, `driveSupplyLimitAmps`, `driveSlipCurrentAmps`, `brownoutShedThresholdVolts`) |
| **`mechanisms.csv`** | Relational Table (`MechanismSpec`) | `robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot` *(+ optional `@DefaultValue` columns: `followerMotorId=-1,invertFollower=false,motorInverted=false,gearRatio=1.0`)* |
| **`cameras.csv`** | Relational Table (`CameraSpec`) | `robot,cameraName,portIndex,xMeters,yMeters,zMeters,rollDeg,pitchDeg,yawDeg,stdDevScalar` |
| **`batteries.csv`** | Relational Table (`BatterySpec`) | `batteryName,qrCode,purchaseYear,nominalRintMilliOhms,maxAllowedRintMilliOhms` |
| **`releases.csv`** | Release Ledger (`ReleaseEntry`) | `version,configSha256,timestamp,author,notes` *(excluded from altering active `ConfigSHA256`)* |
| **Any `<season_table>.csv`** | Schema-Free Relational Table | Arbitrary column headers mapped by name to season Java `record` classes via `bindTable`, `getRowByKey`, and `interpolateTable` |
| **`fluxfig-sheet.properties`** | Google Sheet Sync Config | `sheetId=<SpreadsheetID_or_URL>` and optional `tabs=parameters,mechanisms,cameras,batteries,releases` |
| **`active-profile.txt`** | `FluxTray` Hot-Reload Directive | `<profile>\nmode=PRODUCTION\|TUNING\nConfigSHA256=<8-hex>` (checked by `reloadIfChanged()`) |

*(Note: Parameter keys in `parameters.csv`, `getString`/`getDouble`/`getInt`/`getBoolean`, and `setOverride` are matched case- and underscore-insensitively via `normalizeIdentifier(key)`—e.g., `"controllerProfile"`, `"ControllerProfile"`, and `"controller_profile"` resolve to the same entry while preserving the canonical key name in snapshots and CSV write-backs.)*

---

## Internal Package Architecture (`frc.lib.fluxfig`)

`FluxFig` keeps [`FluxFig`](src/main/java/frc/lib/fluxfig/FluxFig.java) as the single public API entry point for 100% consumer compatibility while delegating specialized responsibilities to focused package-private collaborators:

| Class | Visibility | Responsibility |
| :--- | :--- | :--- |
| [`FluxFig.java`](src/main/java/frc/lib/fluxfig/FluxFig.java) | `public final` | Public facade, profile hierarchy resolution (`<Profile>:<Overlay> -> <Profile> -> DEFAULT`), parameter & spec getters, Two-Mode (`PRODUCTION` vs. `TUNING`) dashboard polling, and version promotion (`promoteSnapshotToDirectory`) |
| [`FluxFigCsvCodec.java`](src/main/java/frc/lib/fluxfig/FluxFigCsvCodec.java) | package-private | RFC-4180 CSV parser, deterministic cross-OS `ConfigSHA256`, `.wpilog` snapshot serialization/re-hydration (`parseEffectiveSnapshot`), `active-profile.txt` directive codec, and `#`-comment/header-preserving CSV serializers |
| [`FluxFigRecordBinder.java`](src/main/java/frc/lib/fluxfig/FluxFigRecordBinder.java) | package-private | Reflective Java `record` binder (`@DefaultValue`, `@RowKey`, `@Tunable`), `@RowKey` preset lookup (`findRowByKey`), and piecewise linear numeric interpolation (`interpolateRows`) |
| [`FluxFigValidator.java`](src/main/java/frc/lib/fluxfig/FluxFigValidator.java) | package-private | Strict hardware safety validator (`validateStrict()`): `60 A` stator/slip & `40 A` supply caps, SystemCore odd CAN FD bus blocking (`can_s1`, `can_s3`), cross-profile CAN ID collision checks, and schema/numeric integrity |
| [`FluxFigSheetSync.java`](src/main/java/frc/lib/fluxfig/FluxFigSheetSync.java) | package-private | Google Sheets URL/ID extraction, `fluxfig-sheet.properties` discovery, `HttpClient` CSV export fetching, and atomic `validateStrict()`-gated local CSV sync |
| [`FluxFigMain.java`](src/main/java/frc/lib/fluxfig/FluxFigMain.java) | `public final` | Standalone CLI entry point (`validate`, `pull`, `promote`, `release`, `sha256`, `get`, `tables`, `snapshot`) |
| [`FluxWpiLogCodec.java`](src/main/java/frc/lib/fluxfig/FluxWpiLogCodec.java) | `public final` | Zero-WPILib binary `.wpilog` v1.0 reader & writer shared across `Flux-SystemCore-2027`, `Flux-RoboRIO-2027`, and `FluxProcessor` |
| [`FluxSystemSnapshot.java`](src/main/java/frc/lib/fluxfig/FluxSystemSnapshot.java) | `public final` | Shared 14-metric SystemCore & roboRIO `system-snapshot.txt` capture script, key-value parser, and actionable watch evaluator shared by `FluxTray` and `FluxProcessor` |

---

## Java API Usage (Production vs. Floor Tuning & Distance Interpolation)
```java
// 1. Define a season table record with @RowKey and @Tunable columns
public enum ShotPreset { SHORT, MID, LONG }

public record ShotRow(
    String robot,
    @FluxFig.RowKey ShotPreset preset,
    double distanceMeters,
    @FluxFig.Tunable double shooterRps,
    @FluxFig.Tunable double hoodRot,
    @FluxFig.Tunable double feederRps,
    @FluxFig.DefaultValue("45.0") double indexerRps) {}

// 2. Load from deploy/configs/ (reads active-profile.txt mode=PRODUCTION|TUNING if present)
Path configDir = Filesystem.getDeployDirectory().toPath().resolve("configs");
FluxFig fig = FluxFig.load(configDir, "FUEL");

// 3. Look up by button preset OR interpolate by vision distance from the exact same table
ShotRow midShot = fig.getRequiredRowByKey("range_table", ShotPreset.MID, ShotRow.class);
ShotRow autoShot = fig.interpolateTable("range_table", "distanceMeters", 2.45, ShotRow.class);

// 4. In Mode.TUNING: poll live dashboard slider changes & write /tmp/fluxfig-tuned-snapshot.txt
fig.setTmpTunedSnapshotPath(FluxFig.DEFAULT_TMP_TUNED_SNAPSHOT_PATH);
fig.pollTuning();
fig.lockProductionOnFms(DriverStation.isFMSAttached()); // Auto-locks to PRODUCTION on FMS

// 5. End of tuning session (in FluxTray / Laptop): harvest snapshot & mint next release (e.g. v4)
FluxFig.ReleaseEntry v4 = FluxFig.promoteSnapshotToDirectory(
    harvestedSnapshotText, laptopConfigDir, "drive-team", "Tuned MID/LONG shooting speeds");
```

---

## Building & Running CLI
```bash
# Linux / macOS:
./scripts/build.sh

# Windows:
scripts\build.bat

# Pull & validate tabs from Google Sheets into a robot's deploy/configs/ folder:
java -jar dist/fluxfig.jar pull ../Flux-SystemCore-2027/src/main/deploy/configs <SheetID_or_URL> parameters,mechanisms,cameras,batteries,releases

# Validate a robot's deploy/configs/ folder:
java -jar dist/fluxfig.jar validate ../Flux-SystemCore-2027/src/main/deploy/configs

# Harvest a tuned snapshot (/tmp/fluxfig-tuned-snapshot.txt) into laptop CSVs & mint next vN release:
java -jar dist/fluxfig.jar promote /tmp/fluxfig-tuned-snapshot.txt ../Flux-SystemCore-2027/src/main/deploy/configs drive-team "Tuned MID shot"

# Record a release tag in releases.csv for the current validated ConfigSHA256:
java -jar dist/fluxfig.jar release ../Flux-SystemCore-2027/src/main/deploy/configs auto boris "Baseline shop config"

# Resolve a parameter with fallback inheritance:
java -jar dist/fluxfig.jar get ../Flux-SystemCore-2027/src/main/deploy/configs COMP_2027:roborio_fallback swerveCanBus

# List discovered relational tables and row counts:
java -jar dist/fluxfig.jar tables ../Flux-SystemCore-2027/src/main/deploy/configs

# Print full deterministic .wpilog snapshot:
java -jar dist/fluxfig.jar snapshot ../Flux-SystemCore-2027/src/main/deploy/configs COMP_2027
```

---

## `FluxTray` Integration Notes (Mode Switching, Upload/Download & Version Promotion)

When `FluxTray.jar` updates its `libs/fluxfig-1.0.0.jar` dependency, it integrates with `FluxFig`'s Two-Mode (`PRODUCTION` vs. `TUNING`) and Version Promotion APIs across four workflows:

1. **Sub-50 ms Mode & Profile Switching (`🔒 Production Mode` $\leftrightarrow$ `🎛️ Tuning Mode`):**
   - `FluxTray` exposes a tray toggle/submenu for **`Mode: PRODUCTION (Locked)`** vs. **`Mode: TUNING (Dashboard Live)`** alongside the active profile selector (`COMP_2027`, `FUEL`, `FUEL:snappy`, etc.).
   - Guarded by `RobotSafetyLockout` (blocked whenever the robot is `Enabled` or connected to `FMS`), `FluxTray` writes `FluxFig.formatActiveProfileDirective(profile, mode, configSha256)` to remote `deploy/configs/active-profile.txt` via `RobotTransport.writeRemoteText(...)`.
   - On the next `disabledPeriodic()` loop, the robot calls `fig.reloadIfChanged()`, switching between `Mode.PRODUCTION` and `Mode.TUNING` in `< 50 ms` without restarting the JVM.
2. **Harvesting Tuned Parameters into a New Release (`📥 Harvest Tuned Values -> Create vN+1`):**
   - While in `Mode.TUNING`, the robot writes live parameter and `@FluxFig.Tunable` relational table edits (`range_table.csv`, `elevator_presets.csv`, `parameters.csv`) to `/tmp/fluxfig-tuned-snapshot.txt` (RAM disk) via `fig.pollTuning()` / `fig.writeTunedSnapshot(...)`.
   - When the team clicks **"Harvest Tuned Values -> Create New Version"** in `FluxTray`, `FluxTray` pulls `/tmp/fluxfig-tuned-snapshot.txt` over SFTP (`RobotTransport.readRemoteText("/tmp/fluxfig-tuned-snapshot.txt")`) — requiring zero NetworkTables client dependencies in `FluxTray.jar`.
   - `FluxTray` calls `FluxFig.promoteSnapshotToDirectory(snapshotText, hubConfigsDir, author, notes)` to update the laptop's `03_FluxFig_Configs/*.csv` files in place (preserving `#` comments and `@Profile` headers), run `validateStrict()`, and append `vN+1` (with the new `ConfigSHA256`, timestamp, author, and auto-generated diff summary) to `releases.csv`.
3. **Re-Upload & Lock to `PRODUCTION` After Promotion:**
   - Immediately after `promoteSnapshotToDirectory(...)` succeeds on the laptop, `FluxTray` deletes `/tmp/fluxfig-tuned-snapshot.txt` on the robot and uploads the newly minted `vN+1` CSV bundle to `deploy/configs/` with `active-profile.txt` set to `mode=PRODUCTION`, so the robot immediately runs the locked, versioned `vN+1` release (`vN+1 (<sha256>)` instead of `<oldSha>-MODIFIED`).
   - Selecting any historical release (`v1..vN`) in `FluxTray` (or clicking **"Discard Tuning & Restore Production"**) deletes `/tmp/fluxfig-tuned-snapshot.txt` and pushes the selected clean CSV bundle with `mode=PRODUCTION`.
4. **Post-Run Auto-Pull Safety Net (`PracticeAutoPullCoordinator`):**
   - During post-run log auto-pull ($\ge 3.0\text{ s}$ `Disabled`), `FluxTray` checks if `/tmp/fluxfig-tuned-snapshot.txt` exists on the robot. If present, `FluxTray` archives a copy into the run's log folder (`tuned-snapshot.txt`) alongside `.wpilog` and `system-snapshot.txt`, and prompts the drive crew to either promote the tuned values to `vN+1` or discard them before powering off the robot.

