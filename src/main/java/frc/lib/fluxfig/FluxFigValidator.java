package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.BatterySpec;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Package-private hardware and configuration safety validator ({@link FluxFig#validateStrict()})
 * enforcing 60 A stator / 40 A supply current caps, even-only SystemCore CAN FD buses ({@code
 * can_s0}, {@code can_s2}, {@code can_s4}), cross-profile resolved CAN ID collision checks, and
 * camera/battery bounds.
 */
final class FluxFigValidator {

  private static final Set<String> VALID_GRAVITY_TYPES =
      Set.of("none", "elevatorstatic", "armcosine");

  private static final Set<String> BUILT_IN_RELATIONAL_TABLES =
      Set.of("mechanisms", "cameras", "batteries", "releases");

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

    Set<String> allKnownProfiles = new LinkedHashSet<>();
    for (List<String> headerList : parameterHeaders.values()) {
      allKnownProfiles.addAll(headerList);
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
      if ("DEFAULT".equalsIgnoreCase(prof)) {
        continue;
      }
      String baseRobot = prof.contains(":") ? prof.substring(0, prof.indexOf(':')) : prof;
      Set<String> resolvedBusIdsForProf = new LinkedHashSet<>();
      for (MechanismSpec spec : allMechanismRows) {
        if (!spec.robot().equalsIgnoreCase(prof) && !spec.robot().equalsIgnoreCase(baseRobot)) {
          continue;
        }
        String resolvedBus = fig.resolveEffectiveCanBus(prof, spec.canBus());
        if (isForbiddenOddSystemCoreCanBus(resolvedBus)
            && !isForbiddenOddSystemCoreCanBus(spec.canBus())) {
          errors.add(
              spec.robot()
                  + ":"
                  + spec.mechanism()
                  + " resolves to forbidden odd SystemCore CAN bus "
                  + resolvedBus
                  + " under profile "
                  + prof);
        }
        String rawLeaderKey = spec.robot() + "|" + spec.canBus() + "|" + spec.motorId();
        String resolvedLeaderKey = prof + "|" + resolvedBus + "|" + spec.motorId();
        if (!resolvedBusIdsForProf.add(resolvedLeaderKey)
            && !resolvedBus.equalsIgnoreCase(spec.canBus())
            && seenCanIds.contains(rawLeaderKey)) {
          errors.add(
              "Duplicate resolved CAN ID under profile "
                  + prof
                  + " after CAN bus remapping: "
                  + resolvedLeaderKey);
        }
        if (spec.followerMotorId() >= 0) {
          String resolvedFollowerKey = prof + "|" + resolvedBus + "|" + spec.followerMotorId();
          if (!resolvedBusIdsForProf.add(resolvedFollowerKey)
               && !resolvedBus.equalsIgnoreCase(spec.canBus())) {
            errors.add(
                "Duplicate resolved follower CAN ID under profile "
                    + prof
                    + " after CAN bus remapping: "
                    + resolvedFollowerKey);
          }
        }
      }
    }

    for (Map.Entry<String, Map<String, String>> paramEntry : parameterTable.entrySet()) {
      String key = paramEntry.getKey();
      for (Map.Entry<String, String> cellEntry : paramEntry.getValue().entrySet()) {
        validateParameterCell(
            "parameters.csv", cellEntry.getKey(), key, cellEntry.getValue().trim(), errors);
      }
    }

    for (Map.Entry<FluxFig.OverrideKey, String> overrideEntry : runtimeOverrides.entrySet()) {
      validateParameterCell(
          "runtimeOverrides",
          overrideEntry.getKey().profile(),
          overrideEntry.getKey().key(),
          overrideEntry.getValue().trim(),
          errors);
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

  private static void validateCustomRelationalTableCells(
      String tableLabel, List<Map<String, String>> rows, List<String> errors) {
    if (rows.isEmpty()) {
      return;
    }
    Set<String> headers = rows.get(0).keySet();
    for (String col : headers) {
      boolean looksNumeric = isNumericTypedColumnHeader(col);
      boolean anyBaseNumeric = false;
      if (!looksNumeric) {
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
        validateParameterCell(tableLabel, "row " + rowNum, col, rawVal, errors);
        if (looksNumeric || (anyBaseNumeric && startsWithNumericOrInvalidToken(rawVal))) {
          if (!isFiniteDoubleLiteral(rawVal)) {
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
        || "mechanism".equals(norm)) {
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

  private static boolean startsWithNumericOrInvalidToken(String s) {
    if ("nan".equalsIgnoreCase(s)
        || "infinity".equalsIgnoreCase(s)
        || "-infinity".equalsIgnoreCase(s)
        || "+infinity".equalsIgnoreCase(s)) {
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
    String keyNorm = FluxFigCsvCodec.normalizeIdentifier(key);
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
