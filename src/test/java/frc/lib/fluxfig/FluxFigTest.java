package frc.lib.fluxfig;

import com.sun.net.httpserver.HttpServer;
import frc.lib.fluxfig.FluxFig.ActiveProfileDirective;
import frc.lib.fluxfig.FluxFig.BatterySpec;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.DefaultValue;
import frc.lib.fluxfig.FluxFig.InMemoryTuningBackend;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.fluxfig.FluxFig.Mode;
import frc.lib.fluxfig.FluxFig.ReleaseEntry;
import frc.lib.fluxfig.FluxFig.RowKey;
import frc.lib.fluxfig.FluxFig.Tunable;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * Zero-dependency unit and integration test suite for {@link FluxFig} (run via {@code java -ea
 * frc.lib.fluxfig.FluxFigTest} in {@code ./scripts/build.sh}, {@code scripts\build.bat}, or
 * {@code ./gradlew test}).
 */
public final class FluxFigTest {

  public enum GravityMode {
    NONE,
    ELEVATOR_STATIC,
    ARM_COSINE
  }

  public enum ShotPreset {
    SHORT,
    MID,
    LONG
  }

  public record ElevatorPreset(
      String robot,
      @RowKey String presetName,
      @Tunable double carriageMeters,
      @Tunable double armAngleDeg,
      @DefaultValue("0.0") double wristAngleDeg) {}

  public record ShotRow(
      String robot,
      @RowKey ShotPreset preset,
      double distanceMeters,
      @Tunable double shooterRps,
      @Tunable double hoodRot,
      @Tunable double feederRps,
      @DefaultValue("45.0") double indexerRps) {}

  public record TypedMechRow(String robot, String mechanism, GravityMode gravityType) {}

  public record SeasonRangeRow(
      double distanceMeters, double shooterRps, double hoodRotations, double timeOfFlightSec) {}

  private static final List<Path> TEMP_PATHS = new ArrayList<>();

  private FluxFigTest() {}

  private static Path createTempDir(String prefix) throws IOException {
    Path dir = Files.createTempDirectory(prefix);
    TEMP_PATHS.add(dir);
    return dir;
  }

  private static Path createTempFile(String prefix, String suffix) throws IOException {
    Path file = Files.createTempFile(prefix, suffix);
    TEMP_PATHS.add(file);
    return file;
  }

  private static void cleanupTempPaths() {
    for (Path root : TEMP_PATHS) {
      if (!Files.exists(root)) {
        continue;
      }
      try (Stream<Path> walk = Files.walk(root)) {
        walk.sorted(Comparator.reverseOrder())
            .forEach(
                p -> {
                  try {
                    Files.deleteIfExists(p);
                  } catch (IOException ignored) {
                  }
                });
      } catch (IOException ignored) {
      }
    }
  }

  private static void check(boolean condition, String message) {
    if (!condition) {
      throw new AssertionError(message);
    }
  }

  public static void main(String[] args) throws Exception {
    try {
      testOneCellControllerFallbackAndCrlfDeterministicSha256();
      testStrictHardwareSafetyValidationAcrossMechanismsParametersAndCameras();
      testSchemaFreeRecordBindingColumnReorderAndReleasesLedger();
      testLiveOverridesPrecedenceAndEffectiveSnapshot();
      testAtomicCsvWriteBackAndEffectiveSnapshotRoundTrip();
      testGoogleSheetUrlParsingLiveHttpPullValidationRollbackAndOfflineFallback();
      testTuningVsProductionModeDistanceTableInterpolationAndReleaseHarvest();
      testResolvedCanBusCollisionValidationHierarchicalTableOverrideAndBatterySave();
      testSharedFluxSystemSnapshotAndFluxWpiLogCodec();
      testWpiLogLongUptimeAndShortBufferGuards();
      testRoboRio2SnapshotCaptureParsingAndEvaluation();
      testHardwareSafetyValidatorBypassesClosed();
      testMultiConfigPresetsActiveProfileChannelAndTargetedCellTuning();
      System.out.println("FluxFigTest: ALL TESTS PASSED");
    } finally {
      cleanupTempPaths();
    }
  }

  private static void testOneCellControllerFallbackAndCrlfDeterministicSha256() throws Exception {
    Path tmpLf = createTempDir("fluxfig-test-lf");
    String paramsLf =
        """
        key,DEFAULT,@COMP_2027,COMP_2027:roborio_fallback,CORAL
        controllerProfile,ROBORIO_CANIVORE_2026,SYSTEMCORE_2027,ROBORIO_CANIVORE_2026,ROBORIO_CANIVORE_2026
        driveCanBus,Drivetrain,can_s0,Drivetrain,CANdace
        mechCanBus,Mech,can_s2,Mech,rio
        loopPeriodSec,0.020,0.005,0.020,0.020
        """;
    String mechsLf =
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        COMP_2027,DriveFL,1,can_s0,60.0,40.0,60.0,None,0.12,0.0,0.0,0.14,0.118,0.015,0.0,-1000,1000,-1,false,false,6.12
        COMP_2027,Elevator,10,can_s2,60.0,40.0,60.0,Elevator_Static,2.4,0.0,0.1,0.18,0.12,0.02,0.38,0.0,42.0,11,true,true,12.5
        """;
    String camsLf =
        """
        robot,cameraName,portIndex,xMeters,yMeters,zMeters,rollDeg,pitchDeg,yawDeg,stdDevScalar
        COMP_2027,limelightsc0,0,0.285,0.000,0.520,0.0,18.0,0.0,1.00
        """;
    String rangeLf =
        """
        distanceMeters,shooterRps,hoodRotations,timeOfFlightSec
        2.0,34.0,2.40,0.32
        4.0,39.5,6.45,0.54
        """;
    String batsLf =
        """
        batteryName,qrCode,purchaseYear,nominalRintMilliOhms,maxAllowedRintMilliOhms
        2026-B01,FLUX-BAT-2026-B01,2026,15.2,22.0
        """;

    Files.writeString(tmpLf.resolve("parameters.csv"), paramsLf, StandardCharsets.UTF_8);
    Files.writeString(tmpLf.resolve("mechanisms.csv"), mechsLf, StandardCharsets.UTF_8);
    Files.writeString(tmpLf.resolve("cameras.csv"), camsLf, StandardCharsets.UTF_8);
    Files.writeString(tmpLf.resolve("range_table.csv"), rangeLf, StandardCharsets.UTF_8);
    Files.writeString(tmpLf.resolve("batteries.csv"), batsLf, StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpLf);
    check(
        "DEFAULT".equals(fig.getActiveProfile()),
        "Default profile should be DEFAULT, got " + fig.getActiveProfile());
    check(fig.getMode() == Mode.PRODUCTION, "Default mode should be PRODUCTION");
    check("ROBORIO_CANIVORE_2026".equals(fig.getString("controllerProfile")), "Default controller");
    check(fig.getConfigSha256().length() == 8, "Expected 8-char hex SHA-256");
    check(fig.validateStrict().isEmpty(), "Expected 0 validation errors on valid config");

    // Verify Windows CRLF (\r\n) and UTF-8 BOM (\uFEFF) produce the exact same ConfigSHA256 as LF
    Path tmpCrlf = createTempDir("fluxfig-test-crlf");
    Files.writeString(
        tmpCrlf.resolve("parameters.csv"),
        "\uFEFF" + paramsLf.replace("\n", "\r\n"),
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpCrlf.resolve("mechanisms.csv"),
        "\uFEFF" + mechsLf.replace("\n", "\r\n"),
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpCrlf.resolve("cameras.csv"), camsLf.replace("\n", "\r\n"), StandardCharsets.UTF_8);
    Files.writeString(
        tmpCrlf.resolve("range_table.csv"), rangeLf.replace("\n", "\r\n"), StandardCharsets.UTF_8);
    Files.writeString(
        tmpCrlf.resolve("batteries.csv"), batsLf.replace("\n", "\r\n"), StandardCharsets.UTF_8);
    FluxFig figCrlf = FluxFig.load(tmpCrlf);
    check(
        fig.getConfigSha256().equals(figCrlf.getConfigSha256()),
        "CRLF + UTF-8 BOM and LF files must produce identical ConfigSHA256");

    fig.setActiveProfile("COMP_2027");
    check("SYSTEMCORE_2027".equals(fig.getString("controllerProfile")), "COMP_2027 controller");
    check("can_s0".equals(fig.getString("driveCanBus")), "COMP_2027 driveCanBus");
    check(Math.abs(fig.getDouble("loopPeriodSec") - 0.005) < 1e-6, "COMP_2027 loopPeriodSec");
    MechanismSpec scElev = fig.getMechanismSpec("Elevator");
    check("can_s2".equals(scElev.canBus()), "COMP_2027 Elevator uses SystemCore can_s2");
    check(scElev.followerMotorId() == 11, "COMP_2027 Elevator followerMotorId 11");
    check(scElev.invertFollower(), "COMP_2027 Elevator invertFollower true");
    check(scElev.motorInverted(), "COMP_2027 Elevator motorInverted true");
    check(Math.abs(scElev.gearRatio() - 12.5) < 1e-6, "COMP_2027 Elevator gearRatio 12.5");

    // Switching to COMP_2027:roborio_fallback must automatically remap MechanismSpec.canBus()
    // from can_s0/can_s2 to Drivetrain/Mech without duplicating rows in mechanisms.csv!
    fig.setActiveProfile("COMP_2027:roborio_fallback");
    check(
        "ROBORIO_CANIVORE_2026".equals(fig.getString("controllerProfile")),
        "Fallback controllerProfile");
    check("Drivetrain".equals(fig.getString("driveCanBus")), "Fallback driveCanBus");
    check(Math.abs(fig.getDouble("loopPeriodSec") - 0.020) < 1e-6, "Fallback loopPeriodSec");
    check(
        "Drivetrain".equals(fig.getMechanismSpec("DriveFL").canBus()),
        "COMP_2027:roborio_fallback DriveFL must auto-remap can_s0 -> Drivetrain");
    check(
        "Mech".equals(fig.getMechanismSpec("Elevator").canBus()),
        "COMP_2027:roborio_fallback Elevator must auto-remap can_s2 -> Mech");

    // Verify getMechanismSpec and getCamerasForRobot inherit across <Profile>:<Overlay> and @Profile
    check(fig.getMechanismSpec("Elevator").motorId() == 10, "Active profile fallback mechanism");
    check(
        fig.getMechanismSpec("COMP_2027:roborio_fallback", "Elevator").motorId() == 10,
        "Overlay profile fallback mechanism");
    check(
        fig.getMechanismSpec("@COMP_2027", "Elevator").motorId() == 10,
        "@Profile fallback mechanism");
    check(fig.getCamerasForActiveProfile().size() == 1, "Active profile fallback cameras");
    check(
        fig.getCamerasForRobot("COMP_2027:roborio_fallback").size() == 1,
        "Overlay profile fallback cameras");
    check(fig.getCamerasForRobot("@COMP_2027").size() == 1, "@Profile fallback cameras");
    check(fig.getTable("range_table", SeasonRangeRow.class).size() == 2, "Range table rows");
    check(fig.getBatteries().containsKey("2026-B01"), "Battery present");
  }

  private static void testStrictHardwareSafetyValidationAcrossMechanismsParametersAndCameras()
      throws Exception {
    Path tmpDir = createTempDir("fluxfig-test-invalid");
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        """
        key,DEFAULT,COMP_2027
        controllerProfile,SYSTEMCORE_2027,SYSTEMCORE_2027
        swerveCanBus,can_s0,can_s3
        driveStatorLimitAmps,60.0,95.0
        driveStatorLimitAmps,60.0,60.0
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("mechanisms.csv"),
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId
        COMP_2027,BadSlipDrive,1,can_s0,60.0,40.0,120.0,None,0.1,0,0,0,0,0,0,0,1,-1
        COMP_2027,OddBusMech,2,can_s1,60.0,40.0,60.0,None,0.1,0,0,0,0,0,0,0,1,-1
        COMP_2027,DupIdMech,1,can_s0,60.0,40.0,60.0,None,0.1,0,0,0,0,0,0,10,-5,-1
        COMP_2027,DupIdMech,4,can_s0,60.0,40.0,60.0,BadGravityType,0.1,0,0,0,0,0,0,0,5,-1
        COMP_2027,TypoCurrentMech,6,can_s2,60A,40.0,60.0,None,0.1,0,0,0,0,0,0,0,5,-1
        COMP_2027,SelfFollowerMech,8,can_s2,60.0,40.0,60.0,None,0.1,0,0,0,0,0,0,0,5,8
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("cameras.csv"),
        """
        robot,cameraName,portIndex,xMeters,yMeters,zMeters,rollDeg,pitchDeg,yawDeg,stdDevScalar
        COMP_2027,limelightsc0,0,0.28,0.0,0.52,0,15,0,1.0
        COMP_2027,limelightsc1,0,0.24,0.2,0.52,0,15,25,1.0
        COMP_2027,limelightsc4,4,0.20,-0.2,0.52,0,15,-25,1.0
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("batteries.csv"),
        """
        batteryName,qrCode,purchaseYear,nominalRintMilliOhms,maxAllowedRintMilliOhms
        2026-B01,QR-1,2026,15.0,22.0
        2026-B01,QR-1,2026,28.0,22.0
        """,
        StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpDir);
    fig.setOverride("COMP_2027", "overrideStatorLimitAmps", 85.0);
    fig.setOverride("COMP_2027", "overrideCanBus", "can_s1");
    fig.setTableCellOverride("mechanisms", "COMP_2027", "OddBusMech", "kP", 0.25);
    List<String> errors = fig.validateStrict();
    check(
        errors.stream().anyMatch(e -> e.contains("duplicate parameter key 'driveStatorLimitAmps'")),
        "Expected duplicate parameter key error in: " + errors);
    check(
        errors.stream()
            .anyMatch(
                e ->
                    e.contains(
                        "mechanisms.csv [row 5, col 'statorLimitAmps']: invalid number '60A'")),
        "Expected exact row/column parse diagnostic in: " + errors);
    check(
        errors.stream().anyMatch(e -> e.contains("invalid gravityType 'BadGravityType'")),
        "Expected invalid gravityType diagnostic in: " + errors);
    check(
        errors.stream()
            .anyMatch(e -> e.contains("SystemCore requires USB portIndex in [0, 3]")),
        "Expected SystemCore USB portIndex [0, 3] bound check in: " + errors);
    check(
        errors.stream()
            .anyMatch(e -> e.contains("followerMotorId (8) cannot equal leader motorId (8)")),
        "Expected leader == follower motorId check in: " + errors);
    check(
        errors.size() >= 17,
        "Expected at least 17 safety/parse errors, got (" + errors.size() + "): " + errors);
  }

  private static void testSchemaFreeRecordBindingColumnReorderAndReleasesLedger() throws Exception {
    Path tmpDir = createTempDir("fluxfig-test-schemafree");
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        "Key,DEFAULT,COMP_2027\nloopPeriodMs,20,5\n",
        StandardCharsets.UTF_8);
    // Reordered columns in mechanisms.csv + extra notes column to verify header-based binding
    // and @DefaultValue optional fields (followerMotorId=-1, invertFollower=false, motorInverted=false, gearRatio=1.0)
    Files.writeString(
        tmpDir.resolve("mechanisms.csv"),
        """
        mechanism,robot,canBus,motorId,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,notes
        Elevator,COMP_2027,can_s2,10,60.0,40.0,60.0,Elevator_Static,2.4,0.0,0.1,0.18,0.12,0.02,0.38,0.0,42.0,Reordered columns test
        """,
        StandardCharsets.UTF_8);
    // Test snake_case header normalization (carriage_meters -> carriageMeters) and @DefaultValue on wristAngleDeg
    Files.writeString(
        tmpDir.resolve("elevator_presets.csv"),
        """
        robot,preset_name,carriage_meters,arm_angle_deg
        COMP_2027,L1,0.15,10.0
        COMP_2027,L4,1.22,-28.5
        FUEL,STOW,0.0,0.0
        """,
        StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpDir, "COMP_2027");
    String sha = fig.getConfigSha256();
    check(fig.getTableNames().contains("elevator_presets"), "Contains elevator_presets table");
    MechanismSpec elev = fig.getMechanismSpec("COMP_2027", "Elevator");
    check(elev.motorId() == 10, "Elevator motorId 10");
    check(elev.followerMotorId() == -1, "Default followerMotorId -1");
    check(!elev.invertFollower(), "Default invertFollower false");
    check(!elev.motorInverted(), "Default motorInverted false");
    check(Math.abs(elev.gearRatio() - 1.0) < 1e-6, "Default gearRatio 1.0");

    // Verify case-insensitive enum binding ("Elevator_Static" -> GravityMode.ELEVATOR_STATIC)
    List<TypedMechRow> typedMechs = fig.getTable("mechanisms", TypedMechRow.class);
    check(typedMechs.size() == 1, "Expected 1 typed mechanism row");
    check(
        typedMechs.get(0).gravityType() == GravityMode.ELEVATOR_STATIC,
        "Case-insensitive enum binding");

    List<ElevatorPreset> allPresets = fig.getTable("elevator_presets", ElevatorPreset.class);
    check(allPresets.size() == 3, "Expected 3 total rows, got " + allPresets.size());

    List<ElevatorPreset> compPresets =
        fig.getTable("elevator_presets", "COMP_2027:roborio_fallback", ElevatorPreset.class);
    check(compPresets.size() == 2, "Expected 2 COMP_2027 rows, got " + compPresets.size());
    check("L4".equals(compPresets.get(1).presetName()), "L4 preset name");
    check(Math.abs(compPresets.get(1).carriageMeters() - 1.22) < 1e-6, "L4 carriageMeters");
    check(
        Math.abs(compPresets.get(1).wristAngleDeg() - 0.0) < 1e-6,
        "@DefaultValue wristAngleDeg 0.0");

    // Add releases.csv ledger and verify that it matches the existing ConfigSHA256 without
    // changing the ConfigSHA256 itself.
    Files.writeString(
        tmpDir.resolve("releases.csv"),
        "version,configSha256,timestamp,author,notes\n"
            + "v3,"
            + sha
            + ",2026-10-23T01:00:00Z,mentor,\"Validated L1-L4 elevator presets\"\n",
        StandardCharsets.UTF_8);
    fig.reload();
    check(
        sha.equals(fig.getConfigSha256()),
        "releases.csv ledger should not alter active ConfigSHA256");
    check(fig.getMatchingRelease().isPresent(), "Matching release present");
    check(("v3 (" + sha + ")").equals(fig.getEffectiveVersion()), "Effective version matches v3");
  }

  private static void testLiveOverridesPrecedenceAndEffectiveSnapshot() throws Exception {
    Path tmpDir = createTempDir("fluxfig-test-overrides");
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        """
        key,DEFAULT,COMP_2027,COMP_2027:roborio_fallback
        maxLinearSpeedMps,4.5,4.8,4.2
        Drive:kP,0.10,0.22,
        """,
        StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpDir, "COMP_2027");
    check(!fig.isModified(), "Initially unmodified");
    check(Math.abs(fig.getDouble("maxLinearSpeedMps") - 4.8) < 1e-6, "Initial maxLinearSpeedMps");

    // Override on base COMP_2027 profile and a key containing a colon ("Drive:kP")
    fig.setOverride("maxLinearSpeedMps", 5.1);
    fig.setOverride("Drive:kP", 0.28);
    check(fig.isModified(), "Modified after setOverride");
    check(
        Math.abs(fig.getDouble("maxLinearSpeedMps") - 5.1) < 1e-6, "Overridden maxLinearSpeedMps");
    check(Math.abs(fig.getDouble("Drive:kP") - 0.28) < 1e-6, "Overridden Drive:kP");
    check(fig.getEffectiveVersion().endsWith("-MODIFIED"), "Effective version ends with -MODIFIED");

    // Switching to COMP_2027:roborio_fallback should use the specific CSV overlay (4.2) for
    // maxLinearSpeedMps while inheriting the COMP_2027 override (0.28) for Drive:kP (which is
    // blank in the COMP_2027:roborio_fallback column)
    fig.setActiveProfile("COMP_2027:roborio_fallback");
    check(
        Math.abs(fig.getDouble("maxLinearSpeedMps") - 4.2) < 1e-6,
        "Specific overlay CSV cell must take precedence over parent profile override");
    check(
        Math.abs(fig.getDouble("Drive:kP") - 0.28) < 1e-6,
        "Blank overlay cell should inherit parent profile override");

    Map<String, String> diff = fig.getModifiedDiff();
    check("4.8 -> 5.1".equals(diff.get("COMP_2027.maxLinearSpeedMps")), "Diff maxLinearSpeedMps");
    check("0.22 -> 0.28".equals(diff.get("COMP_2027.Drive:kP")), "Diff Drive:kP");

    String snapshot = fig.getEffectiveSnapshot();
    check(
        snapshot.contains("# EffectiveVersion: " + fig.getEffectiveVersion()),
        "Snapshot EffectiveVersion");
    check(
        snapshot.contains("COMP_2027.maxLinearSpeedMps: 4.8 -> 5.1"),
        "Snapshot maxLinearSpeedMps diff");
    check(snapshot.contains("COMP_2027.Drive:kP: 0.22 -> 0.28"), "Snapshot Drive:kP diff");
    check(snapshot.contains("=== parameters.csv ==="), "Snapshot parameters.csv section");

    fig.clearOverrides();
    check(!fig.isModified(), "Unmodified after clearOverrides");
    fig.setActiveProfile("COMP_2027");
    check(Math.abs(fig.getDouble("maxLinearSpeedMps") - 4.8) < 1e-6, "Restored maxLinearSpeedMps");
  }

  private static void testAtomicCsvWriteBackAndEffectiveSnapshotRoundTrip() throws Exception {
    Path tmpDir = createTempDir("fluxfig-test-writeback");
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        """
        # Team 10413 Fleet Parameters
        key,DEFAULT,@COMP_2027
        controllerProfile,ROBORIO_CANIVORE_2026,SYSTEMCORE_2027
        wheelRadiusMeters,0.0508,0.0508
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("mechanisms.csv"),
        """
        # 2027 Mechanisms with custom notes column
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,notes
        COMP_2027,ElevatorLeader,10,can_s2,60.0,40.0,60.0,ELEVATOR_STATIC,2.4,0.0,0.08,0.18,0.115,0.012,0.38,0.0,48.5,Primary cascade stage
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("cameras.csv"),
        """
        # Camera extrinsics solved by ZeroPriorBootstrapCmd
        robot,cameraName,portIndex,xMeters,yMeters,zMeters,rollDeg,pitchDeg,yawDeg,stdDevScalar
        COMP_2027,limelightsc0,0,0.280,0.000,0.510,0.0,15.0,0.0,1.00
        FUEL,OV9281-2,0,0.279,0.000,0.508,0.0,20.0,0.0,1.00
        """,
        StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpDir, "COMP_2027");
    fig.setOverride("wheelRadiusMeters", 0.0496);
    fig.setTableCellOverride("mechanisms", "COMP_2027", "ElevatorLeader", "kP", 2.75);

    String snapshot = fig.getEffectiveSnapshot();
    FluxFig rehydrated = FluxFig.fromEffectiveSnapshot(snapshot);
    check(
        fig.getConfigSha256().equals(rehydrated.getConfigSha256()),
        "Rehydrated ConfigSHA256 must match original");
    check(
        fig.getEffectiveVersion().equals(rehydrated.getEffectiveVersion()),
        "Rehydrated EffectiveVersion must match original");
    check(
        "COMP_2027".equals(rehydrated.getActiveProfile()),
        "Rehydrated ActiveProfile must match original");
    check(
        Math.abs(rehydrated.getDouble("wheelRadiusMeters") - 0.0496) < 1e-6,
        "Rehydrated runtime override must be preserved");
    check(
        Math.abs(rehydrated.getMechanismSpec("ElevatorLeader").kP() - 2.75) < 1e-6,
        "Rehydrated table cell override on ElevatorLeader.kP must be preserved");

    String shaBefore = fig.getConfigSha256();
    fig.persistOverridesToCsv();
    check(!fig.isModified(), "isModified() must be false after persistOverridesToCsv()");
    check(
        !shaBefore.equals(fig.getConfigSha256()),
        "ConfigSHA256 must update after persisting overrides to disk");
    String savedParamsText = Files.readString(tmpDir.resolve("parameters.csv"), StandardCharsets.UTF_8);
    String savedMechsText = Files.readString(tmpDir.resolve("mechanisms.csv"), StandardCharsets.UTF_8);
    check(
        savedParamsText.contains("# Team 10413 Fleet Parameters")
            && savedParamsText.contains("@COMP_2027"),
        "persistOverridesToCsv must preserve # comments and @COMP_2027 header prefix");
    check(
        savedMechsText.contains("# 2027 Mechanisms with custom notes column")
            && savedMechsText.contains("Primary cascade stage"),
        "persistOverridesToCsv must preserve # comments and custom 'notes' column in mechanisms.csv");

    FluxFig reloaded = FluxFig.load(tmpDir, "COMP_2027");
    check(
        Math.abs(reloaded.getDouble("wheelRadiusMeters") - 0.0496) < 1e-6,
        "Reloaded parameters.csv must contain persisted wheelRadiusMeters");
    check(
        Math.abs(reloaded.getMechanismSpec("ElevatorLeader").kP() - 2.75) < 1e-6,
        "Reloaded mechanisms.csv must contain persisted ElevatorLeader.kP");

    fig.saveCameraSpecs(
        "COMP_2027",
        List.of(
            new CameraSpec(
                "COMP_2027", "limelightsc0", 0, 0.286, -0.004, 0.518, 0.2, 17.8, -0.4, 1.0),
            new CameraSpec(
                "COMP_2027", "limelightsc1", 1, 0.242, 0.262, 0.509, 0.0, 15.1, 29.8, 1.05)));
    check(
        fig.getCamerasForActiveProfile().size() == 2,
        "Expected 2 updated cameras for COMP_2027");
    check(
        fig.getCamerasForRobot("FUEL").size() == 1,
        "saveCameraSpecs for COMP_2027 must preserve FUEL camera rows");
    check(
        Files.readString(tmpDir.resolve("cameras.csv"), StandardCharsets.UTF_8)
            .contains("# Camera extrinsics solved by ZeroPriorBootstrapCmd"),
        "saveCameraSpecs must preserve leading # comments");

    String shaValid = fig.getConfigSha256();
    boolean unsafeRejected = false;
    try {
      fig.saveMechanismSpecs(
          "COMP_2027",
          List.of(
              new MechanismSpec(
                  "COMP_2027",
                  "ElevatorLeader",
                  10,
                  "can_s2",
                  90.0,
                  40.0,
                  60.0,
                  "ELEVATOR_STATIC",
                  2.4,
                  0.0,
                  0.08,
                  0.18,
                  0.115,
                  0.012,
                  0.38,
                  0.0,
                  48.5)));
    } catch (IllegalStateException expected) {
      unsafeRejected = true;
    }
    check(unsafeRejected, "saveMechanismSpecs must reject >60A stator current limit");
    check(
        shaValid.equals(FluxFig.load(tmpDir).getConfigSha256()),
        "Disk files must remain untouched after rejected saveMechanismSpecs");
  }

  private static void testGoogleSheetUrlParsingLiveHttpPullValidationRollbackAndOfflineFallback()
      throws Exception {
    String fullUrl =
        "https://docs.google.com/spreadsheets/d/1EE9hYxSt8KWo1CK-HWMGAMWv1nWoKaDiLkLBEz3asAk/edit#gid=0";
    check(
        "1EE9hYxSt8KWo1CK-HWMGAMWv1nWoKaDiLkLBEz3asAk"
            .equals(FluxFig.extractSpreadsheetId(fullUrl)),
        "Extract spreadsheet ID");
    URI exportUri = FluxFig.buildSheetCsvExportUri(fullUrl, "elevator_presets");
    check(
        "https://docs.google.com/spreadsheets/d/1EE9hYxSt8KWo1CK-HWMGAMWv1nWoKaDiLkLBEz3asAk/gviz/tq?tqx=out:csv&sheet=elevator_presets"
            .equals(exportUri.toString()),
        "Export URI");

    Map<String, String> hostedTabs = new LinkedHashMap<>();
    hostedTabs.put(
        "parameters.csv",
        """
        key,DEFAULT,COMP_2027
        controllerProfile,ROBORIO_CANIVORE_2026,SYSTEMCORE_2027
        swerveCanBus,Drivetrain,can_s0
        loopPeriodMs,20,5
        """);
    hostedTabs.put(
        "mechanisms.csv",
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot
        COMP_2027,ElevatorLeader,10,can_s2,60.0,40.0,60.0,ELEVATOR_STATIC,2.4,0.0,0.08,0.18,0.115,0.012,0.38,0.0,48.5
        """);
    hostedTabs.put(
        "elevator_presets.csv",
        """
        robot,presetName,carriageMeters,armAngleDeg
        COMP_2027,L4,1.25,-28.0
        """);

    HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
    server.setExecutor(
        r -> {
          Thread t = new Thread(r, "fluxfig-test-http");
          t.setDaemon(true);
          t.start();
        });
    server.createContext(
        "/sheet/",
        exchange -> {
          String path = exchange.getRequestURI().getPath();
          String fileName = path.substring("/sheet/".length());
          String body = hostedTabs.get(fileName);
          if (body == null) {
            exchange.getResponseHeaders().set("Connection", "close");
            exchange.sendResponseHeaders(404, -1);
            exchange.close();
            return;
          }
          byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
          exchange.getResponseHeaders().set("Content-Type", "text/csv; charset=utf-8");
          exchange.getResponseHeaders().set("Connection", "close");
          exchange.sendResponseHeaders(200, bytes.length);
          try (OutputStream os = exchange.getResponseBody()) {
            os.write(bytes);
          }
        });
    server.start();
    try {
      String baseUrl = "http://127.0.0.1:" + server.getAddress().getPort() + "/sheet";
      List<String> tabList = List.of("parameters", "mechanisms", "elevator_presets");

      // A. Direct in-memory load from Google Sheet endpoint
      FluxFig inMem = FluxFig.loadFromGoogleSheet(baseUrl, tabList);
      inMem.setActiveProfile("COMP_2027");
      check(
          "SYSTEMCORE_2027".equals(inMem.getString("controllerProfile")),
          "In-memory Google Sheet controllerProfile");
      check(
          inMem.getTable("elevator_presets", ElevatorPreset.class).size() == 1,
          "In-memory Google Sheet elevator_presets");

      // B. Pull from Google Sheet into local configDir via fluxfig-sheet.properties
      Path syncDir = createTempDir("fluxfig-sheet-sync");
      Files.writeString(
          syncDir.resolve(FluxFig.SHEET_PROPERTIES_FILE),
          "sheetId=" + baseUrl + "\ntabs=parameters,mechanisms,elevator_presets\n",
          StandardCharsets.UTF_8);
      FluxFig pulled = FluxFig.pullFromGoogleSheet(syncDir);
      check(Files.isRegularFile(syncDir.resolve("parameters.csv")), "Pulled parameters.csv");
      check(Files.isRegularFile(syncDir.resolve("mechanisms.csv")), "Pulled mechanisms.csv");
      check(
          Files.isRegularFile(syncDir.resolve("elevator_presets.csv")),
          "Pulled elevator_presets.csv");
      check(
          pulled.getConfigSha256().equals(inMem.getConfigSha256()),
          "Pulled SHA matches in-memory SHA");

      // C. Atomic rollback on safety violation: inject a 120A slip current in the remote Sheet tab
      // and verify pullFromGoogleSheet rejects the update and preserves local CSVs untouched
      String validSha = pulled.getConfigSha256();
      hostedTabs.put(
          "mechanisms.csv",
          """
          robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot
          COMP_2027,ElevatorLeader,10,can_s2,60.0,40.0,120.0,ELEVATOR_STATIC,2.4,0.0,0.08,0.18,0.115,0.012,0.38,0.0,48.5
          """);
      boolean rejected = false;
      try {
        FluxFig.pullFromGoogleSheet(syncDir);
      } catch (IllegalStateException expected) {
        rejected = true;
      }
      check(rejected, "Expected pullFromGoogleSheet to reject 120A slip current in remote sheet");
      FluxFig afterReject = FluxFig.load(syncDir);
      check(
          validSha.equals(afterReject.getConfigSha256()),
          "Local CSVs must remain unmodified when remote Google Sheet fails validateStrict()");

      // D. Offline pit fallback: when remote server is unreachable or returns invalid data,
      // loadWithSheetSync seamlessly loads the local synced CSV directory
      FluxFig offlineFallback =
          FluxFig.loadWithSheetSync(syncDir, "http://127.0.0.1:1/unreachable", tabList);
      check(validSha.equals(offlineFallback.getConfigSha256()), "Offline fallback SHA matches");
    } finally {
      server.stop(0);
    }
  }

  private static void testTuningVsProductionModeDistanceTableInterpolationAndReleaseHarvest()
      throws Exception {
    Path laptopConfigDir = createTempDir("fluxfig-tuning-harvest");
    Files.writeString(
        laptopConfigDir.resolve("parameters.csv"),
        """
        # Operational parameters for FUEL & COMP_2027
        key,DEFAULT,FUEL
        controllerProfile,ROBORIO_CANIVORE_2026,ROBORIO_CANIVORE_2026
        swerveCanBus,Drivetrain,Drivetrain
        IntakeInSpeed,60.0,60.0
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        laptopConfigDir.resolve("range_table.csv"),
        """
        # Unified Shooting Preset & Distance Interpolation Table
        robot,preset,distanceMeters,shooterRps,hoodRot,feederRps,indexerRps
        FUEL,SHORT,1.35,33.3,0.0,40.0,45.0
        FUEL,MID,2.08,35.0,2.8,45.0,45.0
        FUEL,LONG,3.92,39.0,6.8,50.0,50.0
        """,
        StandardCharsets.UTF_8);

    FluxFig prodFig = FluxFig.loadProduction(laptopConfigDir, "FUEL");
    ReleaseEntry v1 = prodFig.recordRelease("v1", "boris", "Initial shooting table");
    check("v1".equals(v1.version()), "Expected v1 release");
    check(
        ("v1 (" + prodFig.getConfigSha256() + ")").equals(prodFig.getEffectiveVersion()),
        "Expected EffectiveVersion v1");

    ShotRow midPreset = prodFig.getRequiredRowByKey("range_table", ShotPreset.MID, ShotRow.class);
    check(Math.abs(midPreset.shooterRps() - 35.0) < 1e-6, "MID preset shooterRps 35.0");
    check(Math.abs(midPreset.hoodRot() - 2.8) < 1e-6, "MID preset hoodRot 2.8");

    // Interpolate halfway between MID (2.08 m, 35.0 RPS, 2.8 rot) and LONG (3.92 m, 39.0 RPS, 6.8 rot)
    // At d = 3.00 m (exact midpoint): shooterRps = 37.0, hoodRot = 4.8
    ShotRow interpMidLong =
        prodFig.interpolateTable("range_table", "distanceMeters", 3.00, ShotRow.class);
    check(
        Math.abs(interpMidLong.shooterRps() - 37.0) < 1e-6,
        "Expected interpolated shooterRps=37.0 at 3.00m, got " + interpMidLong.shooterRps());
    check(
        Math.abs(interpMidLong.hoodRot() - 4.8) < 1e-6,
        "Expected interpolated hoodRot=4.8 at 3.00m, got " + interpMidLong.hoodRot());

    InMemoryTuningBackend dashboard = new InMemoryTuningBackend();
    Path tmpSnapshotFile = createTempFile("fluxfig-tuned-snapshot", ".txt");
    prodFig.setTmpTunedSnapshotPath(tmpSnapshotFile);

    Files.writeString(
        laptopConfigDir.resolve(FluxFig.ACTIVE_PROFILE_FILE),
        FluxFig.formatActiveProfileDirective("FUEL", Mode.TUNING, prodFig.getConfigSha256()),
        StandardCharsets.UTF_8);
    prodFig.enableTuning(dashboard);
    prodFig.reload();
    check(prodFig.getMode() == Mode.TUNING, "Expected Mode.TUNING from active-profile.txt");

    // Bind range_table in TUNING mode -> automatically publishes @Tunable columns to dashboard
    prodFig.bindTable("range_table", "FUEL", ShotRow.class);
    Map<String, String> topics = dashboard.getPublishedTopics();
    check(
        "35.0".equals(topics.get("/FluxFig/Tuning/range_table/MID/shooterRps")),
        "Expected published /FluxFig/Tuning/range_table/MID/shooterRps=35.0");
    check(
        "60.0".equals(topics.get("/FluxFig/Tuning/parameters/IntakeInSpeed")),
        "Expected published /FluxFig/Tuning/parameters/IntakeInSpeed=60.0");
    check(
        !topics.containsKey("/FluxFig/Tuning/parameters/controllerProfile"),
        "Structural parameter controllerProfile must NOT be published as tunable");

    dashboard.putFromDashboard("range_table/MID/shooterRps", 36.5);
    dashboard.putFromDashboard("range_table/MID/hoodRot", 3.1);
    dashboard.putFromDashboard("parameters/IntakeInSpeed", 64.0);
    int changed = prodFig.pollTuning();
    check(changed == 3, "Expected 3 tuned changes polled, got " + changed);
    check(prodFig.isModified(), "Expected isModified() == true after tuning poll");

    // Both button preset (MID) and continuous distance interpolation (3.00m) must immediately reflect 36.5 RPS!
    ShotRow tunedMid = prodFig.getRequiredRowByKey("range_table", ShotPreset.MID, ShotRow.class);
    check(Math.abs(tunedMid.shooterRps() - 36.5) < 1e-6, "Tuned MID shooterRps must be 36.5");
    check(Math.abs(tunedMid.hoodRot() - 3.1) < 1e-6, "Tuned MID hoodRot must be 3.1");
    ShotRow tunedInterp =
        prodFig.interpolateTable("range_table", "distanceMeters", 3.00, ShotRow.class);
    // Midpoint between 36.5 and 39.0 is 37.75 RPS
    check(
        Math.abs(tunedInterp.shooterRps() - 37.75) < 1e-6,
        "Interpolated shooterRps at 3.00m must immediately update to 37.75, got "
            + tunedInterp.shooterRps());

    Files.deleteIfExists(laptopConfigDir.resolve(FluxFig.ACTIVE_PROFILE_FILE));
    String harvestedSnapshot = Files.readString(tmpSnapshotFile, StandardCharsets.UTF_8);
    ReleaseEntry v2 =
        FluxFig.promoteSnapshotToDirectory(
            harvestedSnapshot, laptopConfigDir, "drive-team", "Tuned MID shot on floor");
    check("v2".equals(v2.version()), "Expected auto-incremented v2 release, got " + v2.version());
    check(
        v2.notes().contains("range_table[FUEL:MID].shooterRps: 35.0 -> 36.5"),
        "Expected auto-diff summary in v2 release notes: " + v2.notes());

    // Verify locked PRODUCTION reload on v2
    FluxFig v2Prod = FluxFig.loadProduction(laptopConfigDir, "FUEL");
    check(!v2Prod.isModified(), "Promoted v2 must be unmodified");
    check(
        ("v2 (" + v2.configSha256() + ")").equals(v2Prod.getEffectiveVersion()),
        "Effective version must be v2 (" + v2.configSha256() + ")");
    check(
        Math.abs(
                v2Prod
                    .getRequiredRowByKey("range_table", ShotPreset.MID, ShotRow.class)
                    .shooterRps()
                    - 36.5)
            < 1e-6,
        "Persisted v2 MID shooterRps must be 36.5");
    check(
        Math.abs(v2Prod.getDouble("IntakeInSpeed") - 64.0) < 1e-6,
        "Persisted v2 IntakeInSpeed must be 64.0");

    // and blocks any subsequent setOverride / setTableCellOverride calls while locked in PRODUCTION
    v2Prod.enableTuning(dashboard);
    v2Prod.setOverride("IntakeInSpeed", 99.0);
    check(v2Prod.getMode() == Mode.TUNING && v2Prod.isModified(), "Pre-FMS tuning active");
    v2Prod.lockProductionOnFms(true);
    check(
        v2Prod.getMode() == Mode.PRODUCTION && !v2Prod.isModified(),
        "lockProductionOnFms(true) must force Mode.PRODUCTION and discard overrides");
    check(
        Math.abs(v2Prod.getDouble("IntakeInSpeed") - 64.0) < 1e-6,
        "FMS lock must restore committed v2 value 64.0");
    boolean blockedParamOverride = false;
    try {
      v2Prod.setOverride("IntakeInSpeed", 88.0);
    } catch (IllegalStateException expected) {
      blockedParamOverride = true;
    }
    check(blockedParamOverride, "Locked Mode.PRODUCTION must reject setOverride");
    boolean blockedCellOverride = false;
    try {
      v2Prod.setTableCellOverride("range_table", "MID", "shooterRps", 42.0);
    } catch (IllegalStateException expected) {
      blockedCellOverride = true;
    }
    check(blockedCellOverride, "Locked Mode.PRODUCTION must reject setTableCellOverride");

    // Verify that while FMS lock is active, active-profile.txt with mode=TUNING or setMode(Mode.TUNING)
    // cannot unlock PRODUCTION mode until lockProductionOnFms(false) is called
    Files.writeString(
        laptopConfigDir.resolve(FluxFig.ACTIVE_PROFILE_FILE),
        FluxFig.formatActiveProfileDirective("FUEL", Mode.TUNING, v2Prod.getConfigSha256()),
        StandardCharsets.UTF_8);
    v2Prod.reload();
    check(
        v2Prod.getMode() == Mode.PRODUCTION,
        "FMS-locked instance must stay in Mode.PRODUCTION even if active-profile.txt requests TUNING");
    v2Prod.setMode(Mode.TUNING);
    check(
        v2Prod.getMode() == Mode.PRODUCTION,
        "FMS-locked instance must ignore setMode(Mode.TUNING)");
    v2Prod.lockProductionOnFms(false);

    v2Prod.enableTuning(dashboard);
    v2Prod.setTableCellOverride("range_table", "MID", "shooterRps", "36.5rps_typo");
    List<String> customTableErrors = v2Prod.validateStrict();
    check(
        customTableErrors.stream()
            .anyMatch(
                e ->
                    e.contains(
                        "range_table.csv [row 2, col 'shooterRps']: invalid number '36.5rps_typo'")),
        "validateStrict() must catch invalid numeric cell in custom season table range_table.csv: "
            + customTableErrors);
    String corruptSnapshot = v2Prod.getEffectiveSnapshot();
    v2Prod.clearOverrides();
    boolean promoteRejected = false;
    try {
      FluxFig.promoteSnapshotToDirectory(
          corruptSnapshot, laptopConfigDir, "drive-team", "Should fail on corrupt cell");
    } catch (IllegalStateException expected) {
      promoteRejected = true;
    }
    check(
        promoteRejected,
        "promoteSnapshotToDirectory must reject snapshot with invalid numeric cell in custom table");
  }

  private static void
      testResolvedCanBusCollisionValidationHierarchicalTableOverrideAndBatterySave()
          throws Exception {
    Path tmpDir = createTempDir("fluxfig-test-crossbus");
    // On COMP_2027 (SystemCore), mechCanBus=can_s2 and auxCanBus=can_s4 (separate buses).
    // On COMP_2027:bad_fallback, both mechCanBus and auxCanBus are mistakenly mapped to "rio",
    // causing Elevator (ID 10 on can_s2) and Climber (ID 10 on can_s4) to collide on "rio"!
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        """
        key,DEFAULT,COMP_2027,COMP_2027:bad_fallback
        controllerProfile,SYSTEMCORE_2027,SYSTEMCORE_2027,ROBORIO_CANIVORE_2026
        mechCanBus,can_s2,can_s2,rio
        auxCanBus,can_s4,can_s4,rio
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("mechanisms.csv"),
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot
        COMP_2027,Elevator,10,can_s2,60.0,40.0,60.0,ELEVATOR_STATIC,2.4,0,0,0,0,0,0.38,0,45
        COMP_2027,Climber,10,can_s4,60.0,40.0,60.0,NONE,1.5,0,0,0,0,0,0,0,30
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("elevator_presets.csv"),
        """
        robot,presetName,carriageMeters,armAngleDeg
        DEFAULT,L1,0.15,10.0
        DEFAULT,L4,1.20,-25.0
        COMP_2027,L4,1.28,-30.0
        """,
        StandardCharsets.UTF_8);
    Files.writeString(
        tmpDir.resolve("batteries.csv"),
        """
        # Fleet batteries
        batteryName,qrCode,purchaseYear,nominalRintMilliOhms,maxAllowedRintMilliOhms
        2026-B01,FLUX-BAT-01,2026,15.2,22.0
        """,
        StandardCharsets.UTF_8);

    FluxFig fig = FluxFig.load(tmpDir, "COMP_2027");
    List<String> errors = fig.validateStrict();
    check(
        errors.stream()
            .anyMatch(
                e ->
                    e.contains(
                        "Duplicate resolved CAN ID under profile COMP_2027:bad_fallback after CAN bus remapping: COMP_2027:bad_fallback|rio|10")),
        "validateStrict() must catch cross-profile CAN ID collision after bus remapping: "
            + errors);

    // Fix the fallback bus in parameters.csv so Mech and rio are distinct
    Files.writeString(
        tmpDir.resolve("parameters.csv"),
        """
        key,DEFAULT,COMP_2027,COMP_2027:roborio_fallback
        controllerProfile,SYSTEMCORE_2027,SYSTEMCORE_2027,ROBORIO_CANIVORE_2026
        mechCanBus,can_s2,can_s2,Mech
        auxCanBus,can_s4,can_s4,rio
        """,
        StandardCharsets.UTF_8);
    check(fig.reloadIfChanged(), "reloadIfChanged() must detect updated parameters.csv");
    check(fig.validateStrict().isEmpty(), "Expected 0 errors after fixing fallback CAN bus");

    // Verify hierarchical row override in getTableForProfile: DEFAULT has L1 & L4 (1.20m),
    // COMP_2027 overrides L4 (1.28m) -> result must have 2 rows (L1=0.15m, L4=1.28m), NOT 3!
    List<ElevatorPreset> presets = fig.bindTable("elevator_presets", "COMP_2027", ElevatorPreset.class);
    check(
        presets.size() == 2,
        "Hierarchical table override must merge by RowKey (expected 2 rows, got "
            + presets.size()
            + ")");
    ElevatorPreset l4 =
        fig.getRequiredRowByKey("elevator_presets", "L4", ElevatorPreset.class);
    check(
        Math.abs(l4.carriageMeters() - 1.28) < 1e-6,
        "COMP_2027 L4 override must win over DEFAULT L4 (expected 1.28, got "
            + l4.carriageMeters()
            + ")");

    // Verify saveBatterySpecs updates R_int while preserving # comments
    fig.saveBatterySpecs(
        List.of(
            new BatterySpec("2026-B01", "FLUX-BAT-01", 2026, 16.8, 22.0),
            new BatterySpec("2026-B02", "FLUX-BAT-02", 2026, 14.5, 22.0)));
    check(fig.getBatteries().size() == 2, "Expected 2 batteries after saveBatterySpecs");
    check(
        Math.abs(fig.getBatteries().get("2026-B01").nominalRintMilliOhms() - 16.8) < 1e-6,
        "Updated R_int 16.8 mOhm");
    check(
        Files.readString(tmpDir.resolve("batteries.csv"), StandardCharsets.UTF_8)
            .contains("# Fleet batteries"),
        "saveBatterySpecs must preserve leading # comments");

    // Verify ActiveProfileDirective parser
    ActiveProfileDirective dir =
        FluxFig.parseActiveProfileDirective(
            FluxFig.formatActiveProfileDirective("COMP_2027", Mode.TUNING, fig.getConfigSha256()));
    check("COMP_2027".equals(dir.profile()), "Directive profile");
    check(dir.mode() == Mode.TUNING, "Directive mode TUNING");

    // Verify normalized parameter key lookup (snake_case & PascalCase -> canonical camelCase key)
    check(
        "SYSTEMCORE_2027".equals(fig.getString("ControllerProfile")),
        "Normalized PascalCase parameter lookup");
    check(
        "SYSTEMCORE_2027".equals(fig.getString("controller_profile")),
        "Normalized snake_case parameter lookup");
    check(
        "can_s2".equals(fig.getString("mech_can_bus")),
        "Normalized snake_case mech_can_bus lookup");
  }

  private static void testSharedFluxSystemSnapshotAndFluxWpiLogCodec() throws Exception {
    FluxSystemSnapshot baseline = FluxSystemSnapshot.healthySystemCoreBaseline();
    FluxSystemSnapshot reparsed = FluxSystemSnapshot.parse(baseline.serialize());
    check(reparsed.evaluate().isEmpty(), "Healthy SystemCore snapshot must have 0 warnings");

    Path logFile = createTempFile("fluxfig-wpilog-", ".wpilog");
    try (FluxWpiLogCodec.Writer writer = new FluxWpiLogCodec.Writer(logFile, "FLUX-SHARED")) {
      writer.registerStandardGeometrySchemas(0L);
      int dId = writer.startEntry("/Drive/SpeedMps", "double", "", 0L);
      int arrId = writer.startEntry("/Drive/ModuleCurrents", "double[]", "", 0L);
      int bId = writer.startEntry("/DS/Enabled", "boolean", "", 0L);
      int sId = writer.startEntry("/FluxFig/Version", "string", "", 0L);
      int pId = writer.startEntry("/Odometry/Robot", "struct:Pose2d", "", 0L);
      int t3dId = writer.startEntry("/FuelSim/Fuels", "struct:Translation3d[]", "", 0L);
      writer.appendDouble(dId, 20_000L, 4.25);
      writer.appendDoubleArray(arrId, 20_000L, new double[] {38.0, 39.5, 40.0, 38.5});
      writer.appendBoolean(bId, 20_000L, true);
      writer.appendString(sId, 20_000L, "v1");
      writer.appendPose2d(pId, 20_000L, 1.5, 2.5, 0.5);
      writer.appendTranslation3dArray(t3dId, 20_000L, new double[] {1.2, 2.4, 0.3, 4.5, 5.6, 0.9});
    }

    FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(logFile);
    check("FLUX-SHARED".equals(parsed.extraHeader()), "Expected FLUX-SHARED header");
    check(
        Math.abs(parsed.getDoubleSeries("/Drive/SpeedMps").get(0).value() - 4.25) < 1e-6,
        "Expected /Drive/SpeedMps=4.25");
    check(
        parsed.getDoubleArraySeries("/Drive/ModuleCurrents").get(0).values().length == 4,
        "Expected 4-element double[] series");
    check(
        Math.abs(parsed.getDoubleSeries("/Drive/ModuleCurrents[1]").get(0).value() - 39.5) < 1e-6,
        "Expected indexed /Drive/ModuleCurrents[1]=39.5");
    check(parsed.getBooleanSeries("/DS/Enabled").get(0).value(), "Expected /DS/Enabled=true");
    check("v1".equals(parsed.getStringSeries("/FluxFig/Version").get(0).value()), "Expected v1");
    check(
        Math.abs(parsed.getDoubleSeries("/Odometry/Robot.x").get(0).value() - 1.5) < 1e-6,
        "Expected Pose2d .x=1.5");
    check(
        Math.abs(parsed.getDoubleSeries("/FuelSim/Fuels.count").get(0).value() - 2.0) < 1e-6,
        "Expected struct:Translation3d[] .count=2.0");
    check(
        parsed.getDoubleArraySeries("/FuelSim/Fuels").get(0).values().length == 6
            && Math.abs(parsed.getDoubleArraySeries("/FuelSim/Fuels").get(0).values()[4] - 5.6)
                < 1e-6,
        "Expected struct:Translation3d[] 6 unpacked 3D coordinates");

    // Verify FileChannel.read does not hold memory-mapped file locks blocking immediate deletion.
    Files.delete(logFile);
    check(!Files.exists(logFile), "Temp .wpilog must be immediately deletable after read()");

    // Verify absent FluxSystemSnapshot metrics return NaN / NOT MEASURED instead of fake defaults.
    check(
        !FluxSystemSnapshot.REMOTE_CAPTURE_SH_COMMAND.contains("df -BM"),
        "REMOTE_CAPTURE_SH_COMMAND must be BusyBox-safe (no GNU df -BM)");
    FluxSystemSnapshot emptySnap = FluxSystemSnapshot.parse("");
    check("UNKNOWN".equals(emptySnap.controllerType()), "Empty controllerType must be UNKNOWN");
    check("NOT MEASURED".equals(emptySnap.hostname()), "Empty hostname must be NOT MEASURED");
    check(Double.isNaN(emptySnap.cpuTempC()), "Absent cpuTempC must be NaN");
    check(Double.isNaN(emptySnap.totalCpuPct()), "Absent totalCpuPct must be NaN");
    check(Double.isNaN(emptySnap.freeStorageMb()), "Absent freeStorageMb must be NaN");
    check(emptySnap.throttledFlags() == -1, "Absent throttledFlags must be -1");
    check(
        emptySnap.evaluate().isEmpty(),
        "Empty snapshot must not trigger fabricated alerts: " + emptySnap.evaluate());
    check(
        "NOT MEASURED".equals(FluxSystemSnapshot.formatMetric(emptySnap.cpuTempC(), "%.1f", " C")),
        "formatMetric must render NaN as NOT MEASURED");

    // Verify roboRIO snapshot omits ZGC advice on GC loop spikes and preserves real low storage.
    FluxSystemSnapshot rioSnap =
        FluxSystemSnapshot.parse(
            """
            [PLATFORM]
            controllerType=ROBORIO
            hostname=roboRIO-10413-FRC
            [CPU_AND_IRQ]
            totalCpuPct=34.5
            [JVM_AND_STORAGE]
            gcCorrelatedLoopSpikes=2
            freeStorageMb=120.0
            """);
    check(Double.isNaN(rioSnap.cpuTempC()), "Unmeasured roboRIO cpuTempC stays NaN");
    check(
        !rioSnap.serialize().contains("cpuTempC="),
        "serialize() must omit unmeasured NaN fields");
    List<FluxSystemSnapshot.WatchDecision> rioDecisions = rioSnap.evaluate();
    check(rioDecisions.size() == 2, "Expected GC + storage alerts on roboRIO, got " + rioDecisions);
    FluxSystemSnapshot.WatchDecision gcDecision =
        rioDecisions.stream()
            .filter(d -> d.metricName().contains("GC Pause"))
            .findFirst()
            .orElseThrow();
    check(
        !gcDecision.actionableDecision().contains("UseZGC")
            && gcDecision.actionableDecision().contains("UseSerialGC"),
        "roboRIO GC advice must never recommend 64-bit ZGC: " + gcDecision.actionableDecision());
  }

  private static void testWpiLogLongUptimeAndShortBufferGuards() throws Exception {
    Path logFile = createTempFile("fluxfig-wpilog-long-uptime-", ".wpilog");
    long ts1 = 150_000_000_000L;
    long ts2 = 150_002_500_000L;
    try (FluxWpiLogCodec.Writer writer = new FluxWpiLogCodec.Writer(logFile, "LONG-UPTIME")) {
      int loopId = writer.startEntry("/Performance/TotalLoopMs", "double", "", ts1);
      int enabledId = writer.startEntry("/DriverStation/Enabled", "boolean", "", ts1);
      writer.appendDouble(loopId, ts1, 8.25);
      writer.appendDouble(loopId, ts2, 12.75);
      writer.appendBoolean(enabledId, ts1, true);
    }

    FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(logFile);
    check(
        parsed.getDoubleSeries("/Performance/TotalLoopMs").size() == 2,
        "Samples beyond 27.77 h uptime (150e9 us) must not be dropped");
    check(
        parsed.minTimestampUs() == ts1 && parsed.maxTimestampUs() == ts2,
        "Expected exact min/max timestamps at 150e9 us, got "
            + parsed.minTimestampUs()
            + ".."
            + parsed.maxTimestampUs());
    check(
        Math.abs(parsed.durationSeconds() - 2.5) < 1e-6,
        "Expected 2.5 s duration, got " + parsed.durationSeconds());
    check(
        parsed.getBooleanSeries("/DriverStation/Enabled").size() == 1
            && parsed.getBooleanSeries("/DriverStation/Enabled").get(0).value(),
        "Boolean sample at 150e9 us must be retained");

    for (byte[] shortBytes : new byte[][] {new byte[0], new byte[6], new byte[11]}) {
      boolean threwIo = false;
      try {
        FluxWpiLogCodec.parseBuffer(Path.of("short.wpilog"), ByteBuffer.wrap(shortBytes));
      } catch (IOException expected) {
        threwIo = true;
      }
      check(
          threwIo,
          "parseBuffer() must throw IOException (not BufferUnderflowException) for "
              + shortBytes.length
              + "-byte buffer");
    }
    boolean nullThrewIo = false;
    try {
      FluxWpiLogCodec.parseBuffer(Path.of("null.wpilog"), null);
    } catch (IOException expected) {
      nullThrewIo = true;
    }
    check(nullThrewIo, "parseBuffer() must throw IOException for null buffer");
  }

  private static void testRoboRio2SnapshotCaptureParsingAndEvaluation() throws Exception {
    String cmd = FluxSystemSnapshot.REMOTE_CAPTURE_SH_COMMAND;
    check(
        cmd.contains("s1=") && cmd.contains("s2=") && cmd.contains("sleep 0.2"),
        "REMOTE_CAPTURE_SH_COMMAND must take a delta /proc/stat CPU sample");
    check(
        cmd.contains("top -b -n 1"),
        "REMOTE_CAPTURE_SH_COMMAND must provide BusyBox top fallback when ps -eo is unavailable");
    check(
        cmd.contains("internalFreeStorageMb") && cmd.contains("usbFreeStorageMb"),
        "REMOTE_CAPTURE_SH_COMMAND must report /home/lvuser internal flash and /U/logs USB separately");
    check(
        cmd.contains("re-started"),
        "REMOTE_CAPTURE_SH_COMMAND must parse iproute2 're-started' header");

    // Verify the exact iproute2 're-started' ($1) awk parser on realistic SocketCAN output.
    String sampleIpLinkOutput =
        "3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT qlen 10\n"
            + "    link/can  promiscuity 0 minmtu 0 maxmtu 0\n"
            + "    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0\n"
            + "    re-started bus-errors arbit-lost error-warn error-pass bus-off\n"
            + "    3          5          0          1          0          0\n";
    Process proc =
        new ProcessBuilder(
                "sh",
                "-c",
                "awk '/re-started|restarts/ {col=1; for(i=1;i<=NF;i++) if($i ~ /^re-?start/) {col=i; break} if(getline>0) print $(col)+0; exit}'")
            .start();
    try (OutputStream os = proc.getOutputStream()) {
      os.write(sampleIpLinkOutput.getBytes(StandardCharsets.UTF_8));
    }
    String rstOut = new String(proc.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
    check(proc.waitFor() == 0 && "3".equals(rstOut), "Expected re-started=3, got '" + rstOut + "'");

    // RoboRIO 2.0 snapshot with TotalLoopMs overrun, PhoenixSignalsMs spike, /home/lvuser low flash
    // (hidden behind large USB stick before fix), and CAN re-started=2.
    FluxSystemSnapshot rioOverrunSnap =
        FluxSystemSnapshot.parse(
            """
            [PLATFORM]
            controllerType=ROBORIO
            hostname=roboRIO-10413-FRC
            [LOOP_AND_TIMING]
            loopPeriodMs=20.0
            loopMedianMs=7.80
            loopP99Ms=23.50
            loopMaxMs=31.20
            phoenixSignalsTraceMaxMs=8.40
            commandSchedulerTraceMaxMs=6.10
            robotContainerTraceMaxMs=1.90
            [CPU_AND_IRQ]
            totalCpuPct=52.0
            javaProcessCpuPct=38.0
            [SOCKETCAN]
            can.can0=42000,18000,0,0,0,0,2,ERROR-ACTIVE
            [STORAGE]
            internalFreeStorageMb=110.0
            usbFreeStorageMb=14800.0
            freeStorageMb=110.0
            """);
    check(rioOverrunSnap.isRoboRio(), "Snapshot must identify as roboRIO");
    check(
        rioOverrunSnap.applicableMetricCount() == 8,
        "roboRIO applicable metric count must reflect roboRIO hardware (8), got "
            + rioOverrunSnap.applicableMetricCount());
    check(
        Math.abs(rioOverrunSnap.internalFreeStorageMb() - 110.0) < 1e-6
            && Math.abs(rioOverrunSnap.usbFreeStorageMb() - 14800.0) < 1e-6,
        "Internal and USB storage must both round-trip");
    FluxSystemSnapshot rioRoundTrip = FluxSystemSnapshot.parse(rioOverrunSnap.serialize());
    List<FluxSystemSnapshot.WatchDecision> rioDecisions = rioRoundTrip.evaluate();
    check(
        rioDecisions.stream().anyMatch(d -> d.metricName().contains("Loop Period Jitter")),
        "TotalLoopMs p99=23.5ms on 20ms roboRIO must trigger loop jitter decision: " + rioDecisions);
    check(
        rioDecisions.stream().anyMatch(d -> d.metricName().contains("PhoenixSignalsMs")),
        "PhoenixSignalsMs=8.4ms on 20ms roboRIO must trigger subsystem breakdown: " + rioDecisions);
    check(
        rioDecisions.stream()
            .anyMatch(
                d ->
                    d.category().contains("Storage")
                        && "CRITICAL".equals(d.severity())
                        && d.measuredValue().contains("Internal=110 MB")
                        && d.measuredValue().contains("USB=14800 MB")),
        "Low /home/lvuser flash (110 MB) must trigger CRITICAL even with 14.8 GB USB: "
            + rioDecisions);
    check(
        rioDecisions.stream()
            .anyMatch(
                d ->
                    d.metricName().contains("CAN Error State")
                        && "CRITICAL".equals(d.severity())
                        && d.measuredValue().contains("restarts=2")),
        "CAN re-started=2 must trigger CRITICAL CAN alert: " + rioDecisions);
    check(
        "CRITICAL".equals(rioRoundTrip.verdictSummary(rioDecisions)),
        "Expected CRITICAL verdictSummary");

    // Verify single-loop TotalLoopMs max overrun fires even when p99 is within 20 ms budget.
    FluxSystemSnapshot rioPeakOnly =
        FluxSystemSnapshot.parse(
            """
            [PLATFORM]
            controllerType=ROBORIO
            [LOOP_AND_TIMING]
            loopPeriodMs=20.0
            loopMedianMs=6.50
            loopP99Ms=14.00
            loopMaxMs=29.00
            """);
    List<FluxSystemSnapshot.WatchDecision> peakDecisions = rioPeakOnly.evaluate();
    check(
        peakDecisions.stream().anyMatch(d -> d.metricName().contains("Control Loop Overrun")),
        "Single-loop TotalLoopMs=29ms overrun on 20ms roboRIO must fire alert: " + peakDecisions);

    // Honest NO DATA verdict on completely unmeasured snapshot (never false NOMINAL).
    FluxSystemSnapshot emptySnap = FluxSystemSnapshot.parse("[PLATFORM]\ncontrollerType=ROBORIO\n");
    check(
        "NO DATA".equals(emptySnap.verdictSummary(emptySnap.evaluate())),
        "Unmeasured roboRIO snapshot must return NO DATA verdict, got "
            + emptySnap.verdictSummary(emptySnap.evaluate()));
  }

  private static void testHardwareSafetyValidatorBypassesClosed() throws Exception {
    // Baseline valid multi-robot config matching FUEL + COMP_2027 (including SwerveDriveFL row).
    Map<String, String> validBase = new LinkedHashMap<>();
    validBase.put(
        "parameters.csv",
        """
        key,DEFAULT,COMP_2027,FUEL
        controllerProfile,ROBORIO_CANIVORE_2026,SYSTEMCORE_2027,ROBORIO_CANIVORE_2026
        loopPeriodMs,20,5,20
        swerveCanBus,Drivetrain,can_s0,Drivetrain
        mechCanBus,Mech,can_s2,Mech
        auxCanBus,rio,can_s4,rio
        simulateMissingMechanisms,false,false,false
        maxLinearSpeedMps,4.5,4.8,4.5
        maxAngularSpeedRadPerSec,4.71,10.5,4.71
        driveStatorLimitAmps,60.0,60.0,60.0
        driveSupplyLimitAmps,40.0,40.0,40.0
        driveSlipCurrentAmps,60.0,60.0,60.0
        pigeonId,20,20,20
        trackWidthMeters,0.56515,0.56515,0.56515
        wheelBaseMeters,0.56515,0.56515,0.56515
        wheelRadiusMeters,0.0508,0.0508,0.0508
        driveGearRatio,6.3947,6.3947,6.3947
        turnGearRatio,12.1,12.1,12.1
        """);
    validBase.put(
        "mechanisms.csv",
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        FUEL,SwerveDriveFL,1,Drivetrain,60.0,40.0,60.0,NONE,0.90,0.0,0.0,0.14,0.79,0.10,0.0,-10000.0,10000.0,-1,false,false,6.3947
        FUEL,Shooter,11,$mechCanBus,60.0,40.0,60.0,NONE,0.35,0.0,0.0,0.20,0.118,0.018,0.0,-10000.0,10000.0,10,true,true,1.0
        FUEL,Hood,12,Mech,40.0,25.0,40.0,NONE,2.80,0.0,0.05,0.16,0.150,0.010,0.0,0.0,12.5,-1,false,false,1.0
        """);
    validBase.put(
        "swerve_modules.csv",
        """
        robot,corner,driveMotorId,turnMotorId,cancoderId,encoderOffsetRot,invertDrive,invertTurn
        FUEL,FL,2,1,21,0.1052,false,false
        FUEL,FR,4,3,22,-0.1206,true,false
        FUEL,BL,6,5,23,-0.4667,false,false
        FUEL,BR,8,7,24,-0.0371,true,false
        """);
    validBase.put(
        "range_table.csv",
        """
        distanceMeters,shooterRps,hoodRotations,timeOfFlightSec
        1.22,33.33,0.00,0.22
        2.84,36.67,3.90,0.40
        4.67,40.00,8.85,0.62
        """);
    FluxFig baseFig = FluxFig.fromCsvMap(validBase, "FUEL");
    List<String> baseErrors = baseFig.validateStrict();
    check(baseErrors.isEmpty(), "Valid multi-table baseline must pass validateStrict: " + baseErrors);

    // Bypass (1): NaN / Infinity in numeric PID/FF gains and parameters.
    Map<String, String> nanMechs = new LinkedHashMap<>(validBase);
    nanMechs.put(
        "mechanisms.csv",
        validBase.get("mechanisms.csv").replace("0.35,0.0,0.0", "NaN,0.0,Infinity"));
    List<String> nanErrors = FluxFig.fromCsvMap(nanMechs, "FUEL").validateStrict();
    check(
        nanErrors.stream().anyMatch(e -> e.contains("non-finite")),
        "validateStrict() must reject NaN/Infinity in mechanism PID/FF gains: " + nanErrors);

    Map<String, String> nanParam = new LinkedHashMap<>(validBase);
    nanParam.put(
        "parameters.csv", validBase.get("parameters.csv").replace("4.5,4.8,4.5", "4.5,NaN,4.5"));
    FluxFig nanParamFig = FluxFig.fromCsvMap(nanParam, "COMP_2027");
    check(
        nanParamFig.validateStrict().stream().anyMatch(e -> e.contains("non-finite")),
        "validateStrict() must reject NaN in parameters.csv");
    boolean getDoubleRejectedNaN = false;
    try {
      nanParamFig.getDouble("maxLinearSpeedMps");
    } catch (IllegalArgumentException expected) {
      getDoubleRejectedNaN = true;
    }
    check(getDoubleRejectedNaN, "getDouble() must reject NaN");

    // Bypass (2): Invalid boolean typos (motorInverted=ture, invertFollower=1, simulateMissingMechanisms=ture).
    Map<String, String> badBoolMechs = new LinkedHashMap<>(validBase);
    badBoolMechs.put(
        "mechanisms.csv", validBase.get("mechanisms.csv").replace("10,true,true", "10,1,ture"));
    List<String> badBoolMechErrors = FluxFig.fromCsvMap(badBoolMechs, "FUEL").validateStrict();
    check(
        badBoolMechErrors.stream().anyMatch(e -> e.contains("invalid boolean")),
        "validateStrict() must reject invalid boolean typos in mechanisms.csv: "
            + badBoolMechErrors);

    Map<String, String> badBoolParams = new LinkedHashMap<>(validBase);
    badBoolParams.put(
        "parameters.csv",
        validBase.get("parameters.csv").replace("false,false,false", "false,false,ture"));
    FluxFig badBoolParamFig = FluxFig.fromCsvMap(badBoolParams, "FUEL");
    check(
        badBoolParamFig.validateStrict().stream().anyMatch(e -> e.contains("invalid boolean")),
        "validateStrict() must reject invalid boolean typo in parameters.csv");
    boolean getBoolRejectedTypo = false;
    try {
      badBoolParamFig.getBoolean("simulateMissingMechanisms");
    } catch (IllegalArgumentException expected) {
      getBoolRejectedTypo = true;
    }
    check(getBoolRejectedTypo, "getBoolean() must reject 'ture' instead of silently returning false");

    // Bypass (3): $mechCanBus vs literal "Mech" CAN ID collision (both row orders).
    Map<String, String> refThenLiteral = new LinkedHashMap<>(validBase);
    refThenLiteral.put(
        "mechanisms.csv",
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        FUEL,Shooter,11,$mechCanBus,60.0,40.0,60.0,NONE,0.35,0.0,0.0,0.20,0.118,0.018,0.0,-10000.0,10000.0,-1,false,false,1.0
        FUEL,Hood,11,Mech,40.0,25.0,40.0,NONE,2.80,0.0,0.05,0.16,0.150,0.010,0.0,0.0,12.5,-1,false,false,1.0
        """);
    List<String> refLitErrors = FluxFig.fromCsvMap(refThenLiteral, "FUEL").validateStrict();
    check(
        refLitErrors.stream()
            .anyMatch(e -> e.contains("Duplicate resolved CAN ID") && e.contains("FUEL|Mech|11")),
        "validateStrict() must catch $mechCanBus vs literal 'Mech' CAN ID collision: "
            + refLitErrors);

    Map<String, String> literalThenRef = new LinkedHashMap<>(validBase);
    literalThenRef.put(
        "mechanisms.csv",
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        FUEL,Hood,11,Mech,40.0,25.0,40.0,NONE,2.80,0.0,0.05,0.16,0.150,0.010,0.0,0.0,12.5,-1,false,false,1.0
        FUEL,Shooter,11,$mechCanBus,60.0,40.0,60.0,NONE,0.35,0.0,0.0,0.20,0.118,0.018,0.0,-10000.0,10000.0,-1,false,false,1.0
        """);
    List<String> litRefErrors = FluxFig.fromCsvMap(literalThenRef, "FUEL").validateStrict();
    check(
        litRefErrors.stream()
            .anyMatch(e -> e.contains("Duplicate resolved CAN ID") && e.contains("FUEL|Mech|11")),
        "validateStrict() must catch literal 'Mech' vs $mechCanBus CAN ID collision regardless of order: "
            + litRefErrors);

    // Bypass (4a): swerve_modules.csv invalid corners, CAN ID collisions, pigeonId collision, and
    // cross-table collision against mechanisms.csv.
    Map<String, String> badSwerve = new LinkedHashMap<>(validBase);
    badSwerve.put(
        "mechanisms.csv",
        validBase.get("mechanisms.csv")
            + "FUEL,RogueClimber,4,Drivetrain,50.0,35.0,50.0,NONE,1.0,0,0,0,0,0,0,0,10,-1,false,false,1.0\n");
    badSwerve.put(
        "swerve_modules.csv",
        """
        robot,corner,driveMotorId,turnMotorId,cancoderId,encoderOffsetRot,invertDrive,invertTurn
        FUEL,FL,2,1,20,0.1052,false,yes
        FUEL,FR,4,3,65,-0.1206,true,false
        FUEL,XX,2,5,23,-0.4667,false,false
        FUEL,BR,8,7,24,-0.0371,true,false
        """);
    List<String> swerveErrors = FluxFig.fromCsvMap(badSwerve, "FUEL").validateStrict();
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("invalid corner 'XX'")),
        "Expected invalid swerve corner error: " + swerveErrors);
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("missing required swerve corner 'BL'")),
        "Expected missing swerve corner BL error: " + swerveErrors);
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("cancoderId must be in [0, 62]")),
        "Expected out-of-range cancoderId error: " + swerveErrors);
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("collides with pigeonId=20")),
        "Expected swerve CAN ID vs pigeonId collision error: " + swerveErrors);
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("Cross-table CAN ID collision")),
        "Expected cross-table mechanisms.csv vs swerve_modules.csv collision error: " + swerveErrors);
    check(
        swerveErrors.stream().anyMatch(e -> e.contains("invalid boolean 'yes'")),
        "Expected strict boolean check on swerve_modules.csv invertTurn: " + swerveErrors);

    // Bypass (4b): range_table.csv non-monotonic distances and unsafe shooterRps > 45.0.
    Map<String, String> badRange = new LinkedHashMap<>(validBase);
    badRange.put(
        "range_table.csv",
        """
        distanceMeters,shooterRps,hoodRotations,timeOfFlightSec
        2.50,35.0,2.0,0.30
        2.00,60.0,-0.5,0.40
        """);
    List<String> rangeErrors = FluxFig.fromCsvMap(badRange, "FUEL").validateStrict();
    check(
        rangeErrors.stream().anyMatch(e -> e.contains("strictly increasing")),
        "validateStrict() must reject non-monotonic range_table.csv distances: " + rangeErrors);
    check(
        rangeErrors.stream().anyMatch(e -> e.contains("shooterRps") && e.contains("45.0")),
        "validateStrict() must reject unsafe range_table.csv shooterRps > 45.0: " + rangeErrors);
    check(
        rangeErrors.stream().anyMatch(e -> e.contains("hoodRotations")),
        "validateStrict() must reject negative range_table.csv hoodRotations: " + rangeErrors);

    // Bypass (5): Invalid physical constants in parameters.csv and unresolved $typoCanBus.
    Map<String, String> badPhysicalAndRef = new LinkedHashMap<>(validBase);
    badPhysicalAndRef.put(
        "parameters.csv",
        validBase
            .get("parameters.csv")
            .replace("loopPeriodMs,20,5,20", "loopPeriodMs,20,5,0")
            .replace("driveGearRatio,6.3947,6.3947,6.3947", "driveGearRatio,6.3947,-1.0,6.3947")
            .replace("wheelRadiusMeters,0.0508,0.0508,0.0508", "wheelRadiusMeters,0.0,0.0508,0.0508")
            .replace("maxLinearSpeedMps,4.5,4.8,4.5", "maxLinearSpeedMps,4.5,4.8,-2.0")
            .replace("pigeonId,20,20,20", "pigeonId,20,20,70"));
    badPhysicalAndRef.put(
        "mechanisms.csv",
        validBase.get("mechanisms.csv").replace("$mechCanBus", "$typoCanBus"));
    List<String> physErrors = FluxFig.fromCsvMap(badPhysicalAndRef, "FUEL").validateStrict();
    check(
        physErrors.stream().anyMatch(e -> e.contains("loopPeriodMs")),
        "validateStrict() must reject loopPeriodMs <= 0: " + physErrors);
    check(
        physErrors.stream().anyMatch(e -> e.contains("driveGearRatio")),
        "validateStrict() must reject driveGearRatio <= 0: " + physErrors);
    check(
        physErrors.stream().anyMatch(e -> e.contains("wheelRadiusMeters")),
        "validateStrict() must reject wheelRadiusMeters <= 0: " + physErrors);
    check(
        physErrors.stream().anyMatch(e -> e.contains("maxLinearSpeedMps")),
        "validateStrict() must reject maxLinearSpeedMps <= 0: " + physErrors);
    check(
        physErrors.stream().anyMatch(e -> e.contains("pigeonId must be in [0, 62]")),
        "validateStrict() must reject out-of-range pigeonId: " + physErrors);
    check(
        physErrors.stream().anyMatch(e -> e.contains("Unresolved CAN bus reference '$typoCanBus'")),
        "validateStrict() must reject unresolved $typoCanBus reference: " + physErrors);
  }

  private static void testMultiConfigPresetsActiveProfileChannelAndTargetedCellTuning()
      throws Exception {
    check(
        "/tmp/fluxfig-tuned-snapshot.txt".equals(FluxFig.DEFAULT_TMP_TUNED_SNAPSHOT_REMOTE_PATH),
        "Canonical POSIX tuned snapshot remote path must use forward slashes");

    String formatted =
        FluxFig.formatActiveProfileDirective(
            "CHASSIS_3", Mode.TUNING, "aa7a7262", "EXPERIMENTAL", "01_crawl_1p5mps");
    ActiveProfileDirective parsed = FluxFig.parseActiveProfileDirective(formatted);
    check("CHASSIS_3".equals(parsed.profile()), "Parsed profile CHASSIS_3");
    check(parsed.mode() == Mode.TUNING, "Parsed mode TUNING");
    check("aa7a7262".equals(parsed.expectedConfigSha256()), "Parsed ConfigSHA256");
    check("EXPERIMENTAL".equals(parsed.channel()), "Parsed channel EXPERIMENTAL");
    check("01_crawl_1p5mps".equals(parsed.preset()), "Parsed preset name");

    ActiveProfileDirective legacyDir =
        FluxFig.parseActiveProfileDirective("FUEL\nmode=PRODUCTION\nConfigSHA256=12345678\n");
    check("STABLE".equals(legacyDir.channel()), "Legacy PRODUCTION defaults channel to STABLE");
    check("STABLE".equals(legacyDir.preset()), "Legacy PRODUCTION defaults preset to STABLE");

    String baseParams =
        """
        # Base fleet parameters
        key,DEFAULT,FUEL,CHASSIS_3
        controllerProfile,ROBORIO_CANIVORE_2026,ROBORIO_CANIVORE_2026,ROBORIO_CANIVORE_2026
        swerveCanBus,Drivetrain,Drivetrain,rio
        pigeonId,20,20,24
        maxLinearSpeedMps,4.5,,4.2
        maxAngularSpeedRadPerSec,4.71,,
        driveStatorLimitAmps,60.0,,
        driveSupplyLimitAmps,40.0,,
        driveSlipCurrentAmps,60.0,,
        """;
    String crawlPreset =
        """
        # Crawl preset for first carpet run
        key,DEFAULT
        maxLinearSpeedMps,1.5
        maxAngularSpeedRadPerSec,2.5
        joystickSensitivity,0.65
        """;
    String mergedParams =
        FluxFig.mergeBehavioralPresetParametersCsv(baseParams, crawlPreset, "CHASSIS_3");
    String baseMechs =
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        CHASSIS_3,SwerveDriveFL,1,rio,60.0,40.0,60.0,NONE,0.90,0.0,0.0,0.14,0.79,0.10,0.0,-10000.0,10000.0,-1,false,false,6.3947
        """;
    FluxFig mergedFig =
        FluxFig.fromCsvMap(
            Map.of("parameters.csv", mergedParams, "mechanisms.csv", baseMechs), "CHASSIS_3");
    check(
        mergedFig.validateStrict().isEmpty(),
        "Merged behavioral preset must pass validateStrict(): " + mergedFig.validateStrict());
    check(
        Math.abs(mergedFig.getDouble("maxLinearSpeedMps") - 1.5) < 1e-6,
        "Merged preset must override CHASSIS_3 shadowing maxLinearSpeedMps to 1.5");
    check(
        Math.abs(mergedFig.getDouble("maxAngularSpeedRadPerSec") - 2.5) < 1e-6,
        "Merged preset must update maxAngularSpeedRadPerSec to 2.5");
    check(
        Math.abs(mergedFig.getDouble("joystickSensitivity") - 0.65) < 1e-6,
        "Merged preset must append new behavioral key joystickSensitivity");
    check(
        "rio".equals(mergedFig.getString("swerveCanBus")) && mergedFig.getInt("pigeonId") == 24,
        "Merged preset must preserve CHASSIS_3 hardware topology (swerveCanBus=rio, pigeonId=24)");
    check(
        mergedParams.contains("# Base fleet parameters"),
        "Merged parameters.csv must preserve leading comments");

    boolean rejectedHardwareKey = false;
    try {
      FluxFig.mergeBehavioralPresetParametersCsv(
          baseParams, "key,DEFAULT\nswerveCanBus,CANdace\n", "CHASSIS_3");
    } catch (IllegalArgumentException expected) {
      rejectedHardwareKey = true;
    }
    check(rejectedHardwareKey, "Behavioral preset must reject hardware topology key swerveCanBus");

    // Targeted single-cell tuning on mechanisms.csv SwerveDriveFL.kP and structural guards.
    InMemoryTuningBackend backend = new InMemoryTuningBackend();
    Path tmpSnap = createTempFile("fluxfig-targeted-snap-", ".txt");
    mergedFig.setTmpTunedSnapshotPath(tmpSnap);
    mergedFig.registerTunableParameters("maxLinearSpeedMps", "joystickSensitivity");
    mergedFig.registerTunableTableCell("mechanisms", "SwerveDriveFL", "kP");
    mergedFig.enableTuning(backend);
    check(
        "0.90".equals(backend.getPublishedTopics().get("/FluxFig/Tuning/mechanisms/SwerveDriveFL/kP")),
        "Targeted table cell must publish SwerveDriveFL.kP");
    backend.putFromDashboard("mechanisms/SwerveDriveFL/kP", "1.15");
    backend.putFromDashboard("parameters/maxLinearSpeedMps", "2.0");
    check(mergedFig.pollTuning() == 2, "Expected 2 polled targeted tuning edits");
    check(
        Math.abs(mergedFig.getMechanismSpec("CHASSIS_3", "SwerveDriveFL").kP() - 1.15) < 1e-6,
        "Targeted cell edit must immediately update hydrated MechanismSpec kP");
    String snap = Files.readString(tmpSnap, StandardCharsets.UTF_8);
    check(
        snap.contains("mechanisms[CHASSIS_3:SwerveDriveFL].kP: 0.90 -> 1.15")
            && snap.contains("CHASSIS_3.maxLinearSpeedMps: 1.5 -> 2.0"),
        "Written tuned snapshot must capture both parameter and targeted table cell overrides: "
            + snap);

    boolean rejectedStructuralParam = false;
    try {
      mergedFig.registerTunableParameters("driveStatorLimitAmps");
    } catch (IllegalArgumentException expected) {
      rejectedStructuralParam = true;
    }
    check(rejectedStructuralParam, "registerTunableParameters must reject driveStatorLimitAmps");

    boolean rejectedStructuralCell = false;
    try {
      mergedFig.registerTunableTableCell("mechanisms", "SwerveDriveFL", "statorLimitAmps");
    } catch (IllegalArgumentException expected) {
      rejectedStructuralCell = true;
    }
    check(rejectedStructuralCell, "registerTunableTableCell must reject statorLimitAmps");
  }
}
