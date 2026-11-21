package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.BatterySpec;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * Package-private hardware and configuration safety validator ({@link FluxFig#validateStrict()})
 * enforcing 60 A stator / 40 A supply current caps, finite PID/FF gains, strict boolean parsing,
 * even-only SystemCore CAN FD buses ({@code can_s0}, {@code can_s2}, {@code can_s4}), cross-profile
 * resolved CAN ID collision checks (including {@code $...} bus references, {@code
 * swerve_modules.csv}, and {@code pigeonId}), {@code range_table.csv} bounds/monotonicity, physical
 * parameter invariants, and camera/battery bounds.
 */
final class FluxFigValidator {

  private static final Set<String> VALID_GRAVITY_TYPES =
      Set.of("none", "elevatorstatic", "armcosine");

  private static final Set<String> BUILT_IN_RELATIONAL_TABLES =
      Set.of("mechanisms", "cameras", "batteries", "releases");

  private static final Set<String> PARAMETER_METADATA_COLUMNS =
      Set.of("type", "unit", "units", "description", "notes", "comment", "comments");

  private static final Set<String> POSITIVE_PHYSICAL_PARAM_KEYS =
      Set.of(
          "loopperiodms",
          "loopperiodsec",
          "drivegearratio",
          "turngearratio",
          "wheelradiusmeters",
          "trackwidthmeters",
          "wheelbasemeters",
          "swervecouplingratio",
          "maxlinearspeedmps",
          "maxangularspeedradpersec",
          "brownoutshedthresholdvolts",
          "shoplengthmeters",
          "shopwidthmeters");

  private static final Set<String> BOOLEAN_PARAM_KEYS = Set.of("simulatemissingmechanisms");

  private static final Set<String> VALID_SWERVE_CORNERS = Set.of("FL", "FR", "BL", "BR");

  private static final double MAX_SAFE_RANGE_TABLE_SHOOTER_RPS = 45.0;

  private FluxFigValidator() {}

  static List<String> validateStrict(
      FluxFig fig,
      Map<String, String> rawCsvFiles,
      List<String> parseErrors,
      Map<String, List<String>> parameterHeaders,
      Map<String, Map<String, String>> parameterTable,
      Map<FluxFig.OverrideKey, String> runtimeOverrides,
      Map<String, List<Map<String, String>>> relationalTables,
      Map<String, Class<? extends Record>> registeredTableSchemas,
      List<MechanismSpec> allMechanismRows,
      List<CameraSpec> cameraSpecs,
      List<BatterySpec> allBatteryRows) {
    List<String> errors = new ArrayList<>();
    if (rawCsvFiles.isEmpty()) {
      errors.add("No CSV configuration files loaded");
      return errors;
    }
    errors.addAll(parseErrors);

    Set<String> seenMechanismKeys = new LinkedHashSet<>();
    Set<String> seenCanIds = new LinkedHashSet<>();
    Set<String> reportedUnresolvedBusRefs = new LinkedHashSet<>();

    Set<String> allKnownProfiles = new LinkedHashSet<>();
    for (List<String> headerList : parameterHeaders.values()) {
      for (String header : headerList) {
        if (!isParameterMetadataColumn(header)) {
          allKnownProfiles.add(header);
        }
      }
    }
    for (MechanismSpec spec : allMechanismRows) {
      if (spec.robot() != null && !spec.robot().isBlank()) {
        allKnownProfiles.add(spec.robot());
      }
    }
    for (CameraSpec cam : cameraSpecs) {
      if (cam.robot() != null && !cam.robot().isBlank()) {
        allKnownProfiles.add(cam.robot());
      }
    }
    if (relationalTables.containsKey("swerve_modules")) {
      for (Map<String, String> row : fig.getTable("swerve_modules")) {
        String r =
            FluxFigCsvCodec.normalizeProfileHeader(
                FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "robot", ""));
        if (!r.isEmpty()) {
          allKnownProfiles.add(r);
        }
      }
    }

    for (MechanismSpec spec : allMechanismRows) {
      String id = spec.robot() + ":" + spec.mechanism();
      if (!seenMechanismKeys.add(id)) {
        errors.add(
            "Duplicate mechanism definition on robot " + spec.robot() + ": " + spec.mechanism());
      }
      if (spec.motorId() < 0 || spec.motorId() > 62) {
        errors.add(id + " motorId must be in [0, 62] (got " + spec.motorId() + ")");
      }
      if (spec.followerMotorId() != -1
          && (spec.followerMotorId() < 0 || spec.followerMotorId() > 62)) {
        errors.add(
            id + " followerMotorId must be -1 or in [0, 62] (got " + spec.followerMotorId() + ")");
      }
      if (spec.followerMotorId() >= 0 && spec.followerMotorId() == spec.motorId()) {
        errors.add(
            id
                + " followerMotorId ("
                + spec.followerMotorId()
                + ") cannot equal leader motorId ("
                + spec.motorId()
                + ")");
      }
      if (!Double.isFinite(spec.gearRatio()) || spec.gearRatio() <= 0.0) {
        errors.add(id + " gearRatio must be > 0.0 (got " + spec.gearRatio() + ")");
      }
      if (!Double.isFinite(spec.kP())
          || !Double.isFinite(spec.kI())
          || !Double.isFinite(spec.kD())
          || !Double.isFinite(spec.kS())
          || !Double.isFinite(spec.kV())
          || !Double.isFinite(spec.kA())
          || !Double.isFinite(spec.kG())) {
        errors.add(id + " has non-finite PID/FF gains (kP..kG must be finite)");
      }
      if (!VALID_GRAVITY_TYPES.contains(FluxFigCsvCodec.normalizeIdentifier(spec.gravityType()))) {
        errors.add(
            id
                + " has invalid gravityType '"
                + spec.gravityType()
                + "' (expected NONE, ELEVATOR_STATIC, or ARM_COSINE)");
      }
      if (!Double.isFinite(spec.statorLimitAmps())
          || spec.statorLimitAmps() <= 0.0
          || spec.statorLimitAmps() > 60.0
          || !Double.isFinite(spec.slipCurrentAmps())
          || spec.slipCurrentAmps() <= 0.0
          || spec.slipCurrentAmps() > 60.0) {
        errors.add(
            id
                + " stator/slip current limit must be in (0, 60] A ("
                + spec.statorLimitAmps()
                + "A / "
                + spec.slipCurrentAmps()
                + "A)");
      }
      if (!Double.isFinite(spec.supplyLimitAmps())
          || spec.supplyLimitAmps() <= 0.0
          || spec.supplyLimitAmps() > 40.0) {
        errors.add(
            id + " supply current limit must be in (0, 40] A (" + spec.supplyLimitAmps() + "A)");
      }
      if (!Double.isFinite(spec.minPosRot())
          || !Double.isFinite(spec.maxPosRot())
          || spec.minPosRot() > spec.maxPosRot()) {
        errors.add(
            id
                + " has inverted soft limits: minPosRot ("
                + spec.minPosRot()
                + ") > maxPosRot ("
                + spec.maxPosRot()
                + ")");
      }
      if (isForbiddenOddSystemCoreCanBus(spec.canBus())) {
        errors.add(id + " uses forbidden odd SystemCore SPI-shared CAN bus " + spec.canBus());
      }
      if (spec.canBus() != null && spec.canBus().startsWith("$")) {
        checkUnresolvedBusReference(
            fig, spec.robot(), id, spec.canBus(), reportedUnresolvedBusRefs, errors);
      }
      String busKey = spec.robot() + "|" + spec.canBus() + "|" + spec.motorId();
      if (!seenCanIds.add(busKey)) {
        errors.add("Duplicate CAN ID on same robot/bus: " + busKey);
      }
      if (spec.followerMotorId() >= 0) {
        String followerKey = spec.robot() + "|" + spec.canBus() + "|" + spec.followerMotorId();
        if (!seenCanIds.add(followerKey)) {
          errors.add("Duplicate follower CAN ID on same robot/bus: " + followerKey);
        }
      }
    }

    for (String prof : allKnownProfiles) {
      boolean hasDirectDefaultRows =
          "DEFAULT".equalsIgnoreCase(prof)
              && allMechanismRows.stream().anyMatch(m -> "DEFAULT".equalsIgnoreCase(m.robot()));
      if ("DEFAULT".equalsIgnoreCase(prof) && !hasDirectDefaultRows) {
        continue;
      }
      List<MechanismSpec> effectiveMechs = effectiveMechanismsForProfile(allMechanismRows, prof);
      Map<String, String> resolvedBusIdsForProf = new LinkedHashMap<>();
      for (MechanismSpec spec : effectiveMechs) {
        String id = spec.robot() + ":" + spec.mechanism();
        if (spec.canBus() != null && spec.canBus().startsWith("$")) {
          checkUnresolvedBusReference(
              fig, prof, id, spec.canBus(), reportedUnresolvedBusRefs, errors);
        }
        String resolvedBus = fig.resolveEffectiveCanBus(prof, spec.canBus());
        if (isForbiddenOddSystemCoreCanBus(resolvedBus)
            && !isForbiddenOddSystemCoreCanBus(spec.canBus())) {
          errors.add(
              id
                  + " resolves to forbidden odd SystemCore CAN bus "
                  + resolvedBus
                  + " under profile "
                  + prof);
        }
        String rawLeaderKey = spec.robot() + "|" + spec.canBus() + "|" + spec.motorId();
        String normResolvedLeaderKey =
            prof.toLowerCase(Locale.ROOT)
                + "|"
                + resolvedBus.trim().toLowerCase(Locale.ROOT)
                + "|"
                + spec.motorId();
        String displayResolvedLeaderKey = prof + "|" + resolvedBus + "|" + spec.motorId();
        String firstRawLeaderKey =
            resolvedBusIdsForProf.putIfAbsent(normResolvedLeaderKey, rawLeaderKey);
        if (firstRawLeaderKey != null && !firstRawLeaderKey.equalsIgnoreCase(rawLeaderKey)) {
          errors.add(
              "Duplicate resolved CAN ID under profile "
                  + prof
                  + " after CAN bus remapping: "
                  + displayResolvedLeaderKey);
        }
        if (spec.followerMotorId() >= 0) {
          String rawFollowerKey = spec.robot() + "|" + spec.canBus() + "|" + spec.followerMotorId();
          String normResolvedFollowerKey =
              prof.toLowerCase(Locale.ROOT)
                  + "|"
                  + resolvedBus.trim().toLowerCase(Locale.ROOT)
                  + "|"
                  + spec.followerMotorId();
          String displayResolvedFollowerKey =
              prof + "|" + resolvedBus + "|" + spec.followerMotorId();
          String firstRawFollowerKey =
              resolvedBusIdsForProf.putIfAbsent(normResolvedFollowerKey, rawFollowerKey);
          if (firstRawFollowerKey != null
              && !firstRawFollowerKey.equalsIgnoreCase(rawFollowerKey)) {
            errors.add(
                "Duplicate resolved follower CAN ID under profile "
                    + prof
                    + " after CAN bus remapping: "
                    + displayResolvedFollowerKey);
          }
        }
      }
    }

    for (Map.Entry<String, Map<String, String>> paramEntry : parameterTable.entrySet()) {
      String key = paramEntry.getKey();
      boolean isBooleanParam = isBooleanParameterKey(key, paramEntry.getValue());
      for (Map.Entry<String, String> cellEntry : paramEntry.getValue().entrySet()) {
        if (isParameterMetadataColumn(cellEntry.getKey())) {
          continue;
        }
        validateParameterCell(
            "parameters.csv",
            cellEntry.getKey(),
            key,
            cellEntry.getValue().trim(),
            isBooleanParam,
            errors);
      }
    }

    for (Map.Entry<FluxFig.OverrideKey, String> overrideEntry : runtimeOverrides.entrySet()) {
      String key = overrideEntry.getKey().key();
      Map<String, String> existingCells = parameterTable.get(fig.resolveCanonicalParameterKey(key));
      boolean isBooleanParam = isBooleanParameterKey(key, existingCells);
      validateParameterCell(
          "runtimeOverrides",
          overrideEntry.getKey().profile(),
          key,
          overrideEntry.getValue().trim(),
          isBooleanParam,
          errors);
    }

    validatePigeonVsMechanismsCollisions(fig, allKnownProfiles, allMechanismRows, errors);

    if (relationalTables.containsKey("swerve_modules")) {
      validateSwerveModulesTable(
          fig, allKnownProfiles, allMechanismRows, fig.getTable("swerve_modules"), errors);
    }

    if (relationalTables.containsKey("range_table")) {
      validateRangeTable(fig.getTable("range_table"), errors);
    }

    Set<String> seenCameraNames = new LinkedHashSet<>();
    Set<String> seenCameraPorts = new LinkedHashSet<>();
    for (CameraSpec cam : cameraSpecs) {
      String camId = cam.robot() + ":" + cam.cameraName();
      if (!seenCameraNames.add(cam.robot() + "|" + cam.cameraName())) {
        errors.add("Duplicate cameraName on robot " + cam.robot() + ": " + cam.cameraName());
      }
      if (!seenCameraPorts.add(cam.robot() + "|" + cam.portIndex())) {
        errors.add("Duplicate camera portIndex on robot " + cam.robot() + ": " + cam.portIndex());
      }
      String controller = fig.resolveEffectiveParameter(cam.robot(), "controllerProfile").orElse("");
      boolean isSystemCore = controller.toUpperCase(Locale.ROOT).contains("SYSTEMCORE");
      if (cam.portIndex() < 0 || (isSystemCore && cam.portIndex() > 3)) {
        errors.add(
            camId
                + " has invalid portIndex "
                + cam.portIndex()
                + (isSystemCore ? " (SystemCore requires USB portIndex in [0, 3])" : ""));
      }
      if (!Double.isFinite(cam.xMeters())
          || !Double.isFinite(cam.yMeters())
          || !Double.isFinite(cam.zMeters())
          || !Double.isFinite(cam.rollDeg())
          || !Double.isFinite(cam.pitchDeg())
          || !Double.isFinite(cam.yawDeg())) {
        errors.add(camId + " has non-finite 6-DOF camera transform values");
      }
      if (!Double.isFinite(cam.stdDevScalar()) || cam.stdDevScalar() <= 0.0) {
        errors.add(camId + " has non-positive stdDevScalar: " + cam.stdDevScalar());
      }
    }

    Set<String> seenBatteryNames = new LinkedHashSet<>();
    Set<String> seenBatteryQrCodes = new LinkedHashSet<>();
    for (BatterySpec bat : allBatteryRows) {
      if (!seenBatteryNames.add(bat.batteryName())) {
        errors.add("Duplicate batteryName in batteries.csv: " + bat.batteryName());
      }
      if (!bat.qrCode().isEmpty() && !seenBatteryQrCodes.add(bat.qrCode())) {
        errors.add("Duplicate qrCode in batteries.csv: " + bat.qrCode());
      }
      if (!Double.isFinite(bat.nominalRintMilliOhms())
          || bat.nominalRintMilliOhms() <= 0.0
          || !Double.isFinite(bat.maxAllowedRintMilliOhms())
          || bat.maxAllowedRintMilliOhms() <= 0.0
          || bat.nominalRintMilliOhms() > bat.maxAllowedRintMilliOhms()) {
        errors.add(
            "Invalid R_int thresholds in batteries.csv for "
                + bat.batteryName()
                + ": nominal="
                + bat.nominalRintMilliOhms()
                + " mOhm, maxAllowed="
                + bat.maxAllowedRintMilliOhms()
                + " mOhm");
      }
    }

    for (String tableName : relationalTables.keySet()) {
      String cleanTable = tableName.toLowerCase(Locale.ROOT);
      if (BUILT_IN_RELATIONAL_TABLES.contains(cleanTable)) {
        continue;
      }
      List<Map<String, String>> effectiveRows = fig.getTable(cleanTable);
      Class<? extends Record> schema = registeredTableSchemas.get(cleanTable);
      if (schema != null) {
        FluxFigRecordBinder.bindRowsToRecord(
            cleanTable + ".csv", effectiveRows, schema, true, errors);
      }
      validateCustomRelationalTableCells(cleanTable + ".csv", effectiveRows, errors);
    }

    return errors;
  }

  private static List<MechanismSpec> effectiveMechanismsForProfile(
      List<MechanismSpec> allMechanismRows, String profile) {
    String baseRobot = profile.contains(":") ? profile.substring(0, profile.indexOf(':')) : profile;
    boolean isOverlay = profile.contains(":");
    Set<String> overlayMechanismNames = new LinkedHashSet<>();
    if (isOverlay) {
      for (MechanismSpec spec : allMechanismRows) {
        if (spec.robot().equalsIgnoreCase(profile)) {
          overlayMechanismNames.add(FluxFigCsvCodec.normalizeIdentifier(spec.mechanism()));
        }
      }
    }
    List<MechanismSpec> effective = new ArrayList<>();
    for (MechanismSpec spec : allMechanismRows) {
      if (spec.robot().equalsIgnoreCase(profile)) {
        effective.add(spec);
      } else if (isOverlay
          && spec.robot().equalsIgnoreCase(baseRobot)
          && !overlayMechanismNames.contains(
              FluxFigCsvCodec.normalizeIdentifier(spec.mechanism()))) {
        effective.add(spec);
      }
    }
    return effective;
  }

  private static void checkUnresolvedBusReference(
      FluxFig fig,
      String profile,
      String ownerLabel,
      String rawBus,
      Set<String> reportedKeys,
      List<String> errors) {
    String refKey = rawBus.substring(1).trim();
    Optional<String> resolved =
        refKey.isEmpty() ? Optional.empty() : fig.resolveEffectiveParameter(profile, refKey);
    if (resolved.isEmpty()
        || resolved.get().isBlank()
        || resolved.get().trim().startsWith("$")) {
      String dedupKey = ownerLabel + "|" + rawBus + "|" + profile;
      if (reportedKeys.add(dedupKey)) {
        errors.add(
            "Unresolved CAN bus reference '"
                + rawBus
                + "' for "
                + ownerLabel
                + " under profile "
                + profile);
      }
    }
  }

  private static void validatePigeonVsMechanismsCollisions(
      FluxFig fig,
      Set<String> allKnownProfiles,
      List<MechanismSpec> allMechanismRows,
      List<String> errors) {
    for (String prof : allKnownProfiles) {
      if ("DEFAULT".equalsIgnoreCase(prof)) {
        continue;
      }
      Optional<Integer> pigeonIdOpt = resolveValidCanIdParam(fig, prof, "pigeonId");
      Optional<String> swerveBusOpt = resolveEffectiveSwerveCanBus(fig, prof);
      if (pigeonIdOpt.isEmpty() || swerveBusOpt.isEmpty()) {
        continue;
      }
      int pigeonId = pigeonIdOpt.get();
      String swerveBus = swerveBusOpt.get();
      for (MechanismSpec spec : effectiveMechanismsForProfile(allMechanismRows, prof)) {
        if (isSwerveTemplateMechanism(spec.mechanism())) {
          continue;
        }
        String resolvedBus = fig.resolveEffectiveCanBus(prof, spec.canBus());
        if (resolvedBus != null && resolvedBus.equalsIgnoreCase(swerveBus)) {
          if (spec.motorId() == pigeonId) {
            errors.add(
                "CAN ID collision under profile "
                    + prof
                    + " on bus '"
                    + swerveBus
                    + "': mechanism "
                    + spec.mechanism()
                    + " motorId="
                    + spec.motorId()
                    + " collides with pigeonId="
                    + pigeonId);
          }
          if (spec.followerMotorId() >= 0 && spec.followerMotorId() == pigeonId) {
            errors.add(
                "CAN ID collision under profile "
                    + prof
                    + " on bus '"
                    + swerveBus
                    + "': mechanism "
                    + spec.mechanism()
                    + " followerMotorId="
                    + spec.followerMotorId()
                    + " collides with pigeonId="
                    + pigeonId);
          }
        }
      }
    }
  }

  private static void validateSwerveModulesTable(
      FluxFig fig,
      Set<String> allKnownProfiles,
      List<MechanismSpec> allMechanismRows,
      List<Map<String, String>> rows,
      List<String> errors) {
    if (rows.isEmpty()) {
      return;
    }
    Map<String, Set<String>> cornersByRobot = new LinkedHashMap<>();
    Map<String, Map<Integer, String>> canIdsByRobot = new LinkedHashMap<>();

    for (int i = 0; i < rows.size(); i++) {
      int rowNum = i + 1;
      Map<String, String> row = rows.get(i);
      String robot =
          FluxFigCsvCodec.normalizeProfileHeader(
              FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "robot", ""));
      String cornerRaw = FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "corner", "").trim();
      String cornerUpper = cornerRaw.toUpperCase(Locale.ROOT);
      if (robot.isEmpty()) {
        errors.add("swerve_modules.csv [row " + rowNum + "]: missing robot name");
        continue;
      }
      if (!VALID_SWERVE_CORNERS.contains(cornerUpper)) {
        errors.add(
            "swerve_modules.csv [row "
                + rowNum
                + ", robot "
                + robot
                + "]: invalid corner '"
                + cornerRaw
                + "' (expected FL, FR, BL, or BR)");
      } else if (!cornersByRobot
          .computeIfAbsent(robot, k -> new LinkedHashSet<>())
          .add(cornerUpper)) {
        errors.add(
            "swerve_modules.csv [row "
                + rowNum
                + ", robot "
                + robot
                + "]: duplicate swerve corner '"
                + cornerUpper
                + "'");
      }

      Map<Integer, String> robotCanIds =
          canIdsByRobot.computeIfAbsent(robot, k -> new LinkedHashMap<>());
      validateSwerveCanIdCell(
          row, rowNum, robot, cornerUpper, "driveMotorId", robotCanIds, errors);
      validateSwerveCanIdCell(row, rowNum, robot, cornerUpper, "turnMotorId", robotCanIds, errors);
      validateSwerveCanIdCell(row, rowNum, robot, cornerUpper, "cancoderId", robotCanIds, errors);

      String offsetRaw =
          FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "encoderOffsetRot", "").trim();
      try {
        double offset = Double.parseDouble(offsetRaw);
        if (!Double.isFinite(offset) || offset < -1.0 || offset > 1.0) {
          errors.add(
              "swerve_modules.csv [row "
                  + rowNum
                  + ", "
                  + robot
                  + ":"
                  + cornerUpper
                  + "]: encoderOffsetRot must be finite in [-1.0, 1.0] (got '"
                  + offsetRaw
                  + "')");
        }
      } catch (NumberFormatException e) {
        errors.add(
            "swerve_modules.csv [row "
                + rowNum
                + ", col 'encoderOffsetRot']: invalid number '"
                + offsetRaw
                + "'");
      }

      validateStrictBooleanCell(
          "swerve_modules.csv",
          rowNum,
          "invertDrive",
          FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "invertDrive", "").trim(),
          errors);
      validateStrictBooleanCell(
          "swerve_modules.csv",
          rowNum,
          "invertTurn",
          FluxFigCsvCodec.findCellByColumnNameOrDefault(row, "invertTurn", "").trim(),
          errors);
    }

    for (Map.Entry<String, Set<String>> entry : cornersByRobot.entrySet()) {
      String robot = entry.getKey();
      Set<String> corners = entry.getValue();
      for (String required : VALID_SWERVE_CORNERS) {
        if (!corners.contains(required)) {
          errors.add(
              "swerve_modules.csv [robot "
                  + robot
                  + "]: missing required swerve corner '"
                  + required
                  + "'");
        }
      }
    }

    for (Map.Entry<String, Map<Integer, String>> entry : canIdsByRobot.entrySet()) {
      String robot = entry.getKey();
      Map<Integer, String> swerveCanIds = entry.getValue();
      List<String> matchingProfiles = new ArrayList<>();
      matchingProfiles.add(robot);
      for (String prof : allKnownProfiles) {
        if (prof.contains(":")
            && prof.substring(0, prof.indexOf(':')).equalsIgnoreCase(robot)
            && !matchingProfiles.contains(prof)) {
          matchingProfiles.add(prof);
        }
      }

      for (String prof : matchingProfiles) {
        Optional<Integer> pigeonIdOpt = resolveValidCanIdParam(fig, prof, "pigeonId");
        if (pigeonIdOpt.isPresent()) {
          int pigeonId = pigeonIdOpt.get();
          String swerveOwner = swerveCanIds.get(pigeonId);
          if (swerveOwner != null) {
            errors.add(
                "swerve_modules.csv ["
                    + prof
                    + "]: "
                    + swerveOwner
                    + " CAN ID "
                    + pigeonId
                    + " collides with pigeonId="
                    + pigeonId);
          }
        }

        Optional<String> swerveBusOpt = resolveEffectiveSwerveCanBus(fig, prof);
        if (swerveBusOpt.isEmpty()) {
          continue;
        }
        String swerveBus = swerveBusOpt.get();
        for (MechanismSpec spec : effectiveMechanismsForProfile(allMechanismRows, prof)) {
          if (isSwerveTemplateMechanism(spec.mechanism())) {
            continue;
          }
          String resolvedBus = fig.resolveEffectiveCanBus(prof, spec.canBus());
          if (resolvedBus == null || !resolvedBus.equalsIgnoreCase(swerveBus)) {
            continue;
          }
          String leaderCollision = swerveCanIds.get(spec.motorId());
          if (leaderCollision != null) {
            errors.add(
                "Cross-table CAN ID collision under profile "
                    + prof
                    + " on bus '"
                    + swerveBus
                    + "': mechanisms.csv "
                    + spec.mechanism()
                    + " motorId="
                    + spec.motorId()
                    + " collides with swerve_modules.csv "
                    + leaderCollision);
          }
          if (spec.followerMotorId() >= 0) {
            String followerCollision = swerveCanIds.get(spec.followerMotorId());
            if (followerCollision != null) {
              errors.add(
                  "Cross-table CAN ID collision under profile "
                      + prof
                      + " on bus '"
                      + swerveBus
                      + "': mechanisms.csv "
                      + spec.mechanism()
                      + " followerMotorId="
                      + spec.followerMotorId()
                      + " collides with swerve_modules.csv "
                      + followerCollision);
            }
          }
        }
      }
    }
  }

  private static void validateSwerveCanIdCell(
      Map<String, String> row,
      int rowNum,
      String robot,
      String corner,
      String colName,
      Map<Integer, String> robotCanIds,
      List<String> errors) {
    String raw = FluxFigCsvCodec.findCellByColumnNameOrDefault(row, colName, "").trim();
    try {
      int canId = Integer.parseInt(raw);
      if (canId < 0 || canId > 62) {
        errors.add(
            "swerve_modules.csv [row "
                + rowNum
                + ", "
                + robot
                + ":"
                + corner
                + "]: "
                + colName
                + " must be in [0, 62] (got "
                + canId
                + ")");
        return;
      }
      String owner = corner + "." + colName;
      String existing = robotCanIds.putIfAbsent(canId, owner);
      if (existing != null) {
        errors.add(
            "swerve_modules.csv [row "
                + rowNum
                + ", robot "
                + robot
                + "]: duplicate CAN ID "
                + canId
                + " shared between "
                + existing
                + " and "
                + owner);
      }
    } catch (NumberFormatException e) {
      errors.add(
          "swerve_modules.csv [row "
              + rowNum
              + ", col '"
              + colName
              + "']: invalid integer '"
              + raw
              + "'");
    }
  }

  private static void validateRangeTable(List<Map<String, String>> rows, List<String> errors) {
    if (rows.isEmpty()) {
      return;
    }
    String profileCol = FluxFigRecordBinder.findProfileColumnHeader(rows.get(0));
    Map<String, Double> lastDistanceByProfile = new LinkedHashMap<>();

    for (int i = 0; i < rows.size(); i++) {
      int rowNum = i + 1;
      Map<String, String> row = rows.get(i);
      String profKey =
          profileCol != null
              ? FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, "*"))
              : "*";
      if (profKey.isEmpty()) {
        profKey = "*";
      }

      String distRaw = FluxFigCsvCodec.findCellByColumnName(row, "distanceMeters");
      if (distRaw != null && !distRaw.trim().isEmpty()) {
        try {
          double dist = Double.parseDouble(distRaw.trim());
          if (!Double.isFinite(dist) || dist <= 0.0) {
            errors.add(
                "range_table.csv [row "
                    + rowNum
                    + ", col 'distanceMeters']: must be finite and > 0.0 (got '"
                    + distRaw.trim()
                    + "')");
          } else {
            Double prev = lastDistanceByProfile.put(profKey, dist);
            if (prev != null && dist <= prev) {
              errors.add(
                  "range_table.csv [row "
                      + rowNum
                      + ", col 'distanceMeters']: distances must be strictly increasing ("
                      + dist
                      + " <= previous "
                      + prev
                      + ")");
            }
          }
        } catch (NumberFormatException ignored) {
          // Format error already reported by validateCustomRelationalTableCells.
        }
      } else if (distRaw != null) {
        errors.add(
            "range_table.csv [row "
                + rowNum
                + ", col 'distanceMeters']: missing required distanceMeters value");
      }

      String rpsRaw = FluxFigCsvCodec.findCellByColumnName(row, "shooterRps");
      if (rpsRaw != null && !rpsRaw.trim().isEmpty()) {
        try {
          double rps = Double.parseDouble(rpsRaw.trim());
          if (!Double.isFinite(rps) || rps <= 0.0 || rps > MAX_SAFE_RANGE_TABLE_SHOOTER_RPS) {
            errors.add(
                "range_table.csv [row "
                    + rowNum
                    + ", col 'shooterRps']: must be in (0, "
                    + MAX_SAFE_RANGE_TABLE_SHOOTER_RPS
                    + "] RPS (got "
                    + rpsRaw.trim()
                    + ")");
          }
        } catch (NumberFormatException ignored) {
          // Format error already reported by validateCustomRelationalTableCells.
        }
      }

      String hoodRaw = FluxFigCsvCodec.findCellByColumnName(row, "hoodRotations");
      String hoodColName = "hoodRotations";
      if (hoodRaw == null) {
        hoodRaw = FluxFigCsvCodec.findCellByColumnName(row, "hoodRot");
        hoodColName = "hoodRot";
      }
      if (hoodRaw != null && !hoodRaw.trim().isEmpty()) {
        try {
          double hood = Double.parseDouble(hoodRaw.trim());
          if (!Double.isFinite(hood) || hood < 0.0) {
            errors.add(
                "range_table.csv [row "
                    + rowNum
                    + ", col '"
                    + hoodColName
                    + "']: must be finite and >= 0.0 (got "
                    + hoodRaw.trim()
                    + ")");
          }
        } catch (NumberFormatException ignored) {
        }
      }

      String tofRaw = FluxFigCsvCodec.findCellByColumnName(row, "timeOfFlightSec");
      if (tofRaw != null && !tofRaw.trim().isEmpty()) {
        try {
          double tof = Double.parseDouble(tofRaw.trim());
          if (!Double.isFinite(tof) || tof <= 0.0) {
            errors.add(
                "range_table.csv [row "
                    + rowNum
                    + ", col 'timeOfFlightSec']: must be finite and > 0.0 (got "
                    + tofRaw.trim()
                    + ")");
          }
        } catch (NumberFormatException ignored) {
        }
      }
    }
  }

  private static Optional<Integer> resolveValidCanIdParam(FluxFig fig, String prof, String key) {
    Optional<String> rawOpt = fig.resolveEffectiveParameter(prof, key);
    if (rawOpt.isEmpty() || rawOpt.get().isBlank()) {
      return Optional.empty();
    }
    try {
      int id = Integer.parseInt(rawOpt.get().trim());
      return (id >= 0 && id <= 62) ? Optional.of(id) : Optional.empty();
    } catch (NumberFormatException e) {
      return Optional.empty();
    }
  }

  private static Optional<String> resolveEffectiveSwerveCanBus(FluxFig fig, String prof) {
    Optional<String> raw =
        fig.resolveEffectiveParameter(prof, "swerveCanBus")
            .or(() -> fig.resolveEffectiveParameter(prof, "driveCanBus"));
    if (raw.isEmpty() || raw.get().isBlank()) {
      return Optional.empty();
    }
    String resolved = fig.resolveEffectiveCanBus(prof, raw.get().trim());
    return (resolved == null || resolved.isBlank()) ? Optional.empty() : Optional.of(resolved);
  }

  private static boolean isSwerveTemplateMechanism(String mechanismName) {
    if (mechanismName == null) {
      return false;
    }
    return FluxFigCsvCodec.normalizeIdentifier(mechanismName).startsWith("swervedrive");
  }

  private static boolean isParameterMetadataColumn(String colHeader) {
    return PARAMETER_METADATA_COLUMNS.contains(FluxFigCsvCodec.normalizeIdentifier(colHeader));
  }

  private static boolean isBooleanParameterKey(String key, Map<String, String> cells) {
    String norm = FluxFigCsvCodec.normalizeIdentifier(key);
    if (BOOLEAN_PARAM_KEYS.contains(norm)
        || norm.startsWith("invert")
        || norm.endsWith("inverted")
        || norm.startsWith("enable")
        || norm.endsWith("enabled")
        || norm.startsWith("simulate")) {
      return true;
    }
    if (cells != null) {
      for (Map.Entry<String, String> entry : cells.entrySet()) {
        if (isParameterMetadataColumn(entry.getKey())) {
          continue;
        }
        String v = entry.getValue().trim();
        if ("true".equalsIgnoreCase(v) || "false".equalsIgnoreCase(v)) {
          return true;
        }
      }
    }
    return false;
  }

  private static void validateStrictBooleanCell(
      String tableLabel, int rowNum, String col, String rawVal, List<String> errors) {
    if (!"true".equalsIgnoreCase(rawVal) && !"false".equalsIgnoreCase(rawVal)) {
      errors.add(
          tableLabel
              + " [row "
              + rowNum
              + ", col '"
              + col
              + "']: invalid boolean '"
              + rawVal
              + "' (expected 'true' or 'false')");
    }
  }

  private static void validateCustomRelationalTableCells(
      String tableLabel, List<Map<String, String>> rows, List<String> errors) {
    if (rows.isEmpty()) {
      return;
    }
    boolean isSwerveTable = "swerve_modules.csv".equalsIgnoreCase(tableLabel);
    Set<String> headers = rows.get(0).keySet();
    for (String col : headers) {
      boolean looksNumeric = isNumericTypedColumnHeader(col);
      boolean looksBoolean = !isSwerveTable && isBooleanTypedColumnHeader(col);
      boolean anyBaseNumeric = false;
      if (!looksNumeric && !looksBoolean) {
        for (Map<String, String> row : rows) {
          String v = row.getOrDefault(col, "").trim();
          if (!v.isEmpty() && isFiniteDoubleLiteral(v)) {
            anyBaseNumeric = true;
            break;
          }
        }
      }
      for (int r = 0; r < rows.size(); r++) {
        int rowNum = r + 1;
        Map<String, String> row = rows.get(r);
        String rawVal = row.getOrDefault(col, "").trim();
        if (rawVal.isEmpty()) {
          continue;
        }
        if (!isSwerveTable) {
          validateParameterCell(tableLabel, "row " + rowNum, col, rawVal, looksBoolean, errors);
        }
        if (looksNumeric || (anyBaseNumeric && startsWithNumericOrInvalidToken(rawVal))) {
          if (!isFiniteDoubleLiteral(rawVal)) {
            if (!isSwerveTable) {
              errors.add(
                  tableLabel
                      + " [row "
                      + rowNum
                      + ", col '"
                      + col
                      + "']: invalid number '"
                      + rawVal
                      + "'");
            }
          }
        }
      }
    }
  }

  private static boolean isBooleanTypedColumnHeader(String col) {
    String norm = FluxFigCsvCodec.normalizeIdentifier(col);
    return norm.startsWith("invert")
        || norm.endsWith("inverted")
        || norm.startsWith("enable")
        || norm.endsWith("enabled")
        || norm.startsWith("is");
  }

  private static boolean isNumericTypedColumnHeader(String col) {
    String norm = FluxFigCsvCodec.normalizeIdentifier(col);
    if ("robot".equals(norm)
        || "profile".equals(norm)
        || "preset".equals(norm)
        || "presetname".equals(norm)
        || "name".equals(norm)
        || "notes".equals(norm)
        || "qrcode".equals(norm)
        || "version".equals(norm)
        || "author".equals(norm)
        || "timestamp".equals(norm)
        || "configsha256".equals(norm)
        || "canbus".equals(norm)
        || "gravitytype".equals(norm)
        || "cameraname".equals(norm)
        || "batteryname".equals(norm)
        || "mechanism".equals(norm)
        || "corner".equals(norm)) {
      return false;
    }
    return norm.endsWith("meters")
        || norm.endsWith("rps")
        || norm.endsWith("rpm")
        || norm.endsWith("rot")
        || norm.endsWith("rotations")
        || norm.endsWith("deg")
        || norm.endsWith("degrees")
        || norm.endsWith("rad")
        || norm.endsWith("sec")
        || norm.endsWith("seconds")
        || norm.endsWith("ms")
        || norm.endsWith("amps")
        || norm.endsWith("volts")
        || norm.endsWith("mps")
        || norm.endsWith("scalar")
        || norm.endsWith("ratio")
        || "kp".equals(norm)
        || "ki".equals(norm)
        || "kd".equals(norm)
        || "ks".equals(norm)
        || "kv".equals(norm)
        || "ka".equals(norm)
        || "kg".equals(norm);
  }

  private static boolean isNonFiniteLiteral(String s) {
    return "nan".equalsIgnoreCase(s)
        || "infinity".equalsIgnoreCase(s)
        || "-infinity".equalsIgnoreCase(s)
        || "+infinity".equalsIgnoreCase(s);
  }

  private static boolean startsWithNumericOrInvalidToken(String s) {
    if (isNonFiniteLiteral(s)) {
      return true;
    }
    char c = s.charAt(0);
    return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
  }

  private static boolean isFiniteDoubleLiteral(String s) {
    try {
      double d = Double.parseDouble(s);
      return Double.isFinite(d);
    } catch (NumberFormatException e) {
      return false;
    }
  }

  static void validateParameterCell(
      String sourceLabel, String prof, String key, String val, List<String> errors) {
    validateParameterCell(sourceLabel, prof, key, val, false, errors);
  }

  static void validateParameterCell(
      String sourceLabel,
      String prof,
      String key,
      String val,
      boolean expectBoolean,
      List<String> errors) {
    String keyNorm = FluxFigCsvCodec.normalizeIdentifier(key);
    if (isNonFiniteLiteral(val)) {
      errors.add(sourceLabel + " [" + prof + "." + key + "] non-finite numeric value: " + val);
      return;
    }
    if (isForbiddenOddSystemCoreCanBus(val)) {
      errors.add(
          sourceLabel
              + " ["
              + prof
              + "."
              + key
              + "] uses forbidden odd SystemCore SPI-shared CAN bus "
              + val);
    }
    if (keyNorm.endsWith("canbus") && val.startsWith("$")) {
      errors.add(
          sourceLabel
              + " ["
              + prof
              + "."
              + key
              + "] unresolved/recursive CAN bus reference: "
              + val);
    }
    if (keyNorm.endsWith("statorlimitamps") || keyNorm.endsWith("slipcurrentamps")) {
      try {
        double amps = Double.parseDouble(val);
        if (!Double.isFinite(amps) || amps <= 0.0 || amps > 60.0) {
          errors.add(
              sourceLabel
                  + " ["
                  + prof
                  + "."
                  + key
                  + "] exceeds 60A stator/slip cap ("
                  + amps
                  + "A)");
        }
      } catch (NumberFormatException e) {
        errors.add(sourceLabel + " [" + prof + "." + key + "] invalid numeric current: " + val);
      }
    } else if (keyNorm.endsWith("supplylimitamps")) {
      try {
        double amps = Double.parseDouble(val);
        if (!Double.isFinite(amps) || amps <= 0.0 || amps > 40.0) {
          errors.add(
              sourceLabel
                  + " ["
                  + prof
                  + "."
                  + key
                  + "] exceeds 40A supply cap ("
                  + amps
                  + "A)");
        }
      } catch (NumberFormatException e) {
        errors.add(sourceLabel + " [" + prof + "." + key + "] invalid numeric current: " + val);
      }
    } else if (POSITIVE_PHYSICAL_PARAM_KEYS.contains(keyNorm)) {
      try {
        double d = Double.parseDouble(val);
        if (!Double.isFinite(d) || d <= 0.0) {
          errors.add(
              sourceLabel
                  + " ["
                  + prof
                  + "."
                  + key
                  + "] physical constant must be finite and > 0.0 (got "
                  + val
                  + ")");
        }
      } catch (NumberFormatException e) {
        errors.add(
            sourceLabel + " [" + prof + "." + key + "] invalid physical constant: " + val);
      }
    } else if ("pigeonid".equals(keyNorm)) {
      try {
        int id = Integer.parseInt(val);
        if (id < 0 || id > 62) {
          errors.add(
              sourceLabel
                  + " ["
                  + prof
                  + "."
                  + key
                  + "] pigeonId must be in [0, 62] (got "
                  + id
                  + ")");
        }
      } catch (NumberFormatException e) {
        errors.add(sourceLabel + " [" + prof + "." + key + "] invalid pigeonId integer: " + val);
      }
    } else if (expectBoolean) {
      if (!"true".equalsIgnoreCase(val) && !"false".equalsIgnoreCase(val)) {
        errors.add(
            sourceLabel
                + " ["
                + prof
                + "."
                + key
                + "] invalid boolean '"
                + val
                + "' (expected 'true' or 'false')");
      }
    }
  }

  static boolean isForbiddenOddSystemCoreCanBus(String bus) {
    if (bus == null) {
      return false;
    }
    String norm = FluxFigCsvCodec.normalizeIdentifier(bus);
    return "cans1".equals(norm)
        || "cans3".equals(norm)
        || "can1".equals(norm)
        || "can3".equals(norm);
  }
}
