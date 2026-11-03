package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.ActiveProfileDirective;
import frc.lib.fluxfig.FluxFig.BatterySpec;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.fluxfig.FluxFig.Mode;
import frc.lib.fluxfig.FluxFig.OverrideKey;
import frc.lib.fluxfig.FluxFig.ReleaseEntry;
import frc.lib.fluxfig.FluxFig.TableCellOverrideKey;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Package-private RFC-4180 CSV parser, comment- and header-preserving table serializer,
 * snapshot parser/formatter, identifier normalizer, and deterministic {@code ConfigSHA256} digest
 * calculator for {@link FluxFig}.
 */
final class FluxFigCsvCodec {

  private static final Pattern TABLE_OVERRIDE_HEADER_PATTERN =
      Pattern.compile("^([^\\[]+)\\[([^:]+):([^\\]]+)\\]\\.([^:]+):\\s*(.*?)\\s*->\\s*(.*)$");

  private FluxFigCsvCodec() {}

  static String normalizeLineEndings(String text) {
    if (text == null) {
      return "";
    }
    String withoutBom = text.startsWith("\uFEFF") ? text.substring(1) : text;
    return withoutBom.replace("\r\n", "\n").replace("\r", "\n").stripTrailing() + "\n";
  }

  static String normalizeIdentifier(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\uFEFF", "")
        .replace("@", "")
        .replace("_", "")
        .replace("-", "")
        .replace(" ", "")
        .trim()
        .toLowerCase(Locale.ROOT);
  }

  static String normalizeProfileHeader(String s) {
    if (s == null) {
      return "";
    }
    String t = s.replace("\uFEFF", "").trim();
    return t.startsWith("@") ? t.substring(1).trim() : t;
  }

  static String stripCsvExtension(String name) {
    String t = name.trim();
    if (t.toLowerCase(Locale.ROOT).endsWith(".csv")) {
      return t.substring(0, t.length() - 4);
    }
    return t;
  }

  /** RFC-4180 compatible CSV row parser supporting quoted fields and escaped quotes. */
  static String[] parseCsvLine(String line) {
    List<String> tokens = new ArrayList<>();
    StringBuilder sb = new StringBuilder();
    boolean inQuotes = false;
    for (int i = 0; i < line.length(); i++) {
      char ch = line.charAt(i);
      if (inQuotes) {
        if (ch == '"') {
          if (i + 1 < line.length() && line.charAt(i + 1) == '"') {
            sb.append('"');
            i++;
          } else {
            inQuotes = false;
          }
        } else {
          sb.append(ch);
        }
      } else {
        if (ch == '"') {
          inQuotes = true;
        } else if (ch == ',') {
          tokens.add(sb.toString().trim());
          sb.setLength(0);
        } else {
          sb.append(ch);
        }
      }
    }
    tokens.add(sb.toString().trim());
    return tokens.toArray(new String[0]);
  }

  static String formatCsvCell(String value) {
    if (value == null) {
      return "";
    }
    if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
      return "\"" + value.replace("\"", "\"\"") + "\"";
    }
    return value;
  }

  static List<String> extractLeadingCommentLines(String rawCsv) {
    if (rawCsv == null || rawCsv.isBlank()) {
      return List.of();
    }
    List<String> comments = new ArrayList<>();
    for (String line : normalizeLineEndings(rawCsv).lines().toList()) {
      String trimmed = line.trim();
      if (trimmed.isEmpty()) {
        continue;
      }
      if (trimmed.startsWith("#")) {
        comments.add(trimmed);
      } else {
        break;
      }
    }
    return comments;
  }

  static List<String> extractHeaderColumns(String rawCsv, List<String> defaultHeaders) {
    if (rawCsv != null && !rawCsv.isBlank()) {
      for (String line : normalizeLineEndings(rawCsv).lines().toList()) {
        String trimmed = line.trim();
        if (!trimmed.isEmpty() && !trimmed.startsWith("#")) {
          String[] parsed = parseCsvLine(trimmed);
          if (parsed.length > 0) {
            return new ArrayList<>(Arrays.asList(parsed));
          }
        }
      }
    }
    return new ArrayList<>(defaultHeaders);
  }

  static boolean containsNormalizedHeader(List<String> headers, String targetCol) {
    String norm = normalizeIdentifier(targetCol);
    for (String h : headers) {
      if (normalizeIdentifier(h).equals(norm)) {
        return true;
      }
    }
    return false;
  }

  static void ensureHeaderPresent(List<String> headers, String colName) {
    if (!containsNormalizedHeader(headers, colName)) {
      headers.add(colName);
    }
  }

  static String findCellByColumnName(Map<String, String> row, String componentName) {
    if (row.containsKey(componentName)) {
      return row.get(componentName);
    }
    String normTarget = normalizeIdentifier(componentName);
    for (Map.Entry<String, String> entry : row.entrySet()) {
      if (normalizeIdentifier(entry.getKey()).equals(normTarget)) {
        return entry.getValue();
      }
    }
    return null;
  }

  static String findCellByColumnNameOrDefault(
      Map<String, String> row, String componentName, String defaultValue) {
    String v = findCellByColumnName(row, componentName);
    return v != null ? v : defaultValue;
  }

  static void putCellPreservingHeaderCase(
      Map<String, String> row, List<String> headers, String colName, String value) {
    String norm = normalizeIdentifier(colName);
    for (String existingKey : row.keySet()) {
      if (normalizeIdentifier(existingKey).equals(norm)) {
        row.put(existingKey, value);
        return;
      }
    }
    for (String h : headers) {
      if (normalizeIdentifier(h).equals(norm)) {
        row.put(h, value);
        return;
      }
    }
    row.put(colName, value);
  }

  static String serializeRelationalTable(
      List<String> leadingComments, List<String> headers, List<Map<String, String>> rows) {
    StringBuilder sb = new StringBuilder();
    for (String cmt : leadingComments) {
      sb.append(cmt).append('\n');
    }
    for (int i = 0; i < headers.size(); i++) {
      if (i > 0) {
        sb.append(',');
      }
      sb.append(formatCsvCell(headers.get(i)));
    }
    sb.append('\n');
    for (Map<String, String> row : rows) {
      for (int i = 0; i < headers.size(); i++) {
        if (i > 0) {
          sb.append(',');
        }
        String val = findCellByColumnNameOrDefault(row, headers.get(i), "");
        sb.append(formatCsvCell(val));
      }
      sb.append('\n');
    }
    return sb.toString();
  }

  static String serializeCameraSpecsCsv(
      String rawExistingCsv,
      List<Map<String, String>> existingRows,
      String robot,
      List<CameraSpec> updatedSpecs) {
    String normRobot = normalizeProfileHeader(robot);
    List<String> leadingComments = extractLeadingCommentLines(rawExistingCsv);
    List<String> headers =
        extractHeaderColumns(
            rawExistingCsv,
            List.of(
                "robot",
                "cameraName",
                "portIndex",
                "xMeters",
                "yMeters",
                "zMeters",
                "rollDeg",
                "pitchDeg",
                "yawDeg",
                "stdDevScalar"));

    Map<String, Map<String, String>> existingByRobotAndName = new LinkedHashMap<>();
    List<Map<String, String>> mergedRows = new ArrayList<>();
    for (Map<String, String> row : existingRows) {
      String r = normalizeProfileHeader(findCellByColumnNameOrDefault(row, "robot", ""));
      String name = findCellByColumnNameOrDefault(row, "cameraName", "");
      existingByRobotAndName.put(r.toLowerCase(Locale.ROOT) + "|" + normalizeIdentifier(name), row);
      if (!r.equalsIgnoreCase(normRobot)) {
        mergedRows.add(row);
      }
    }
    for (CameraSpec s : updatedSpecs) {
      String lookupKey =
          normRobot.toLowerCase(Locale.ROOT) + "|" + normalizeIdentifier(s.cameraName());
      Map<String, String> row =
          new LinkedHashMap<>(
              existingByRobotAndName.getOrDefault(lookupKey, Collections.emptyMap()));
      putCellPreservingHeaderCase(row, headers, "robot", normRobot);
      putCellPreservingHeaderCase(row, headers, "cameraName", s.cameraName());
      putCellPreservingHeaderCase(row, headers, "portIndex", String.valueOf(s.portIndex()));
      putCellPreservingHeaderCase(row, headers, "xMeters", String.valueOf(s.xMeters()));
      putCellPreservingHeaderCase(row, headers, "yMeters", String.valueOf(s.yMeters()));
      putCellPreservingHeaderCase(row, headers, "zMeters", String.valueOf(s.zMeters()));
      putCellPreservingHeaderCase(row, headers, "rollDeg", String.valueOf(s.rollDeg()));
      putCellPreservingHeaderCase(row, headers, "pitchDeg", String.valueOf(s.pitchDeg()));
      putCellPreservingHeaderCase(row, headers, "yawDeg", String.valueOf(s.yawDeg()));
      putCellPreservingHeaderCase(row, headers, "stdDevScalar", String.valueOf(s.stdDevScalar()));
      mergedRows.add(row);
    }
    return serializeRelationalTable(leadingComments, headers, mergedRows);
  }

  static String serializeMechanismSpecsCsv(
      String rawExistingCsv,
      List<Map<String, String>> existingRows,
      String robot,
      List<MechanismSpec> updatedSpecs) {
    String normRobot = normalizeProfileHeader(robot);
    List<String> leadingComments = extractLeadingCommentLines(rawExistingCsv);
    List<String> headers =
        extractHeaderColumns(
            rawExistingCsv,
            List.of(
                "robot",
                "mechanism",
                "motorId",
                "canBus",
                "statorLimitAmps",
                "supplyLimitAmps",
                "slipCurrentAmps",
                "gravityType",
                "kP",
                "kI",
                "kD",
                "kS",
                "kV",
                "kA",
                "kG",
                "minPosRot",
                "maxPosRot",
                "followerMotorId",
                "invertFollower",
                "motorInverted",
                "gearRatio"));

    for (MechanismSpec s : updatedSpecs) {
      if (s.followerMotorId() != -1) {
        ensureHeaderPresent(headers, "followerMotorId");
      }
      if (s.invertFollower()) {
        ensureHeaderPresent(headers, "invertFollower");
      }
      if (s.motorInverted()) {
        ensureHeaderPresent(headers, "motorInverted");
      }
      if (Math.abs(s.gearRatio() - 1.0) > 1e-9) {
        ensureHeaderPresent(headers, "gearRatio");
      }
    }

    Map<String, Map<String, String>> existingByRobotAndMech = new LinkedHashMap<>();
    List<Map<String, String>> mergedRows = new ArrayList<>();
    for (Map<String, String> row : existingRows) {
      String r = normalizeProfileHeader(findCellByColumnNameOrDefault(row, "robot", ""));
      String m = findCellByColumnNameOrDefault(row, "mechanism", "");
      existingByRobotAndMech.put(r.toLowerCase(Locale.ROOT) + "|" + normalizeIdentifier(m), row);
      if (!r.equalsIgnoreCase(normRobot)) {
        mergedRows.add(row);
      }
    }
    for (MechanismSpec m : updatedSpecs) {
      String lookupKey =
          normRobot.toLowerCase(Locale.ROOT) + "|" + normalizeIdentifier(m.mechanism());
      Map<String, String> row =
          new LinkedHashMap<>(
              existingByRobotAndMech.getOrDefault(lookupKey, Collections.emptyMap()));
      putCellPreservingHeaderCase(row, headers, "robot", normRobot);
      putCellPreservingHeaderCase(row, headers, "mechanism", m.mechanism());
      putCellPreservingHeaderCase(row, headers, "motorId", String.valueOf(m.motorId()));
      putCellPreservingHeaderCase(row, headers, "canBus", m.canBus());
      putCellPreservingHeaderCase(
          row, headers, "statorLimitAmps", String.valueOf(m.statorLimitAmps()));
      putCellPreservingHeaderCase(
          row, headers, "supplyLimitAmps", String.valueOf(m.supplyLimitAmps()));
      putCellPreservingHeaderCase(
          row, headers, "slipCurrentAmps", String.valueOf(m.slipCurrentAmps()));
      putCellPreservingHeaderCase(row, headers, "gravityType", m.gravityType());
      putCellPreservingHeaderCase(row, headers, "kP", String.valueOf(m.kP()));
      putCellPreservingHeaderCase(row, headers, "kI", String.valueOf(m.kI()));
      putCellPreservingHeaderCase(row, headers, "kD", String.valueOf(m.kD()));
      putCellPreservingHeaderCase(row, headers, "kS", String.valueOf(m.kS()));
      putCellPreservingHeaderCase(row, headers, "kV", String.valueOf(m.kV()));
      putCellPreservingHeaderCase(row, headers, "kA", String.valueOf(m.kA()));
      putCellPreservingHeaderCase(row, headers, "kG", String.valueOf(m.kG()));
      putCellPreservingHeaderCase(row, headers, "minPosRot", String.valueOf(m.minPosRot()));
      putCellPreservingHeaderCase(row, headers, "maxPosRot", String.valueOf(m.maxPosRot()));
      if (containsNormalizedHeader(headers, "followerMotorId")) {
        putCellPreservingHeaderCase(
            row, headers, "followerMotorId", String.valueOf(m.followerMotorId()));
      }
      if (containsNormalizedHeader(headers, "invertFollower")) {
        putCellPreservingHeaderCase(
            row, headers, "invertFollower", String.valueOf(m.invertFollower()));
      }
      if (containsNormalizedHeader(headers, "motorInverted")) {
        putCellPreservingHeaderCase(
            row, headers, "motorInverted", String.valueOf(m.motorInverted()));
      }
      if (containsNormalizedHeader(headers, "gearRatio")) {
        putCellPreservingHeaderCase(row, headers, "gearRatio", String.valueOf(m.gearRatio()));
      }
      mergedRows.add(row);
    }
    return serializeRelationalTable(leadingComments, headers, mergedRows);
  }

  static String serializeBatterySpecsCsv(
      String rawExistingCsv,
      List<Map<String, String>> existingRows,
      List<BatterySpec> updatedBatteries) {
    List<String> leadingComments = extractLeadingCommentLines(rawExistingCsv);
    List<String> headers =
        extractHeaderColumns(
            rawExistingCsv,
            List.of(
                "batteryName",
                "qrCode",
                "purchaseYear",
                "nominalRintMilliOhms",
                "maxAllowedRintMilliOhms"));

    Map<String, Map<String, String>> existingByName = new LinkedHashMap<>();
    for (Map<String, String> row : existingRows) {
      String name = findCellByColumnNameOrDefault(row, "batteryName", "");
      existingByName.put(normalizeIdentifier(name), row);
    }

    List<Map<String, String>> mergedRows = new ArrayList<>();
    Set<String> updatedKeys = new LinkedHashSet<>();
    for (BatterySpec b : updatedBatteries) {
      String normName = normalizeIdentifier(b.batteryName());
      updatedKeys.add(normName);
      Map<String, String> row =
          new LinkedHashMap<>(existingByName.getOrDefault(normName, Collections.emptyMap()));
      putCellPreservingHeaderCase(row, headers, "batteryName", b.batteryName());
      putCellPreservingHeaderCase(row, headers, "qrCode", b.qrCode());
      putCellPreservingHeaderCase(row, headers, "purchaseYear", String.valueOf(b.purchaseYear()));
      putCellPreservingHeaderCase(
          row, headers, "nominalRintMilliOhms", String.valueOf(b.nominalRintMilliOhms()));
      putCellPreservingHeaderCase(
          row, headers, "maxAllowedRintMilliOhms", String.valueOf(b.maxAllowedRintMilliOhms()));
      mergedRows.add(row);
    }
    for (Map<String, String> existingRow : existingRows) {
      String name = findCellByColumnNameOrDefault(existingRow, "batteryName", "");
      if (!updatedKeys.contains(normalizeIdentifier(name))) {
        mergedRows.add(existingRow);
      }
    }
    return serializeRelationalTable(leadingComments, headers, mergedRows);
  }

  static FluxFig parseEffectiveSnapshot(String snapshotText) {
    if (snapshotText == null || snapshotText.isBlank()) {
      throw new IllegalArgumentException("Effective snapshot text must not be blank");
    }
    String normalized = normalizeLineEndings(snapshotText);
    List<String> lines = normalized.lines().toList();

    String parsedActiveProfile = "DEFAULT";
    Mode parsedMode = Mode.PRODUCTION;
    Map<OverrideKey, String> parsedOverrides = new LinkedHashMap<>();
    Map<TableCellOverrideKey, String> parsedTableOverrides = new LinkedHashMap<>();
    Map<String, StringBuilder> fileSections = new LinkedHashMap<>();

    String currentFile = null;
    int headerSection = 0;

    for (String line : lines) {
      String trimmed = line.trim();
      if (trimmed.startsWith("=== ") && trimmed.endsWith(" ===") && trimmed.length() > 8) {
        currentFile = trimmed.substring(4, trimmed.length() - 4).trim();
        if (currentFile.contains("/") || currentFile.contains("\\") || currentFile.contains("..")) {
          throw new IllegalArgumentException("Invalid snapshot section filename: " + currentFile);
        }
        headerSection = 0;
        fileSections.computeIfAbsent(currentFile, k -> new StringBuilder());
        continue;
      }
      if (currentFile == null) {
        if (trimmed.startsWith("# ActiveProfile:")) {
          parsedActiveProfile = trimmed.substring("# ActiveProfile:".length()).trim();
          headerSection = 0;
        } else if (trimmed.startsWith("# Mode:")) {
          String rawMode = trimmed.substring("# Mode:".length()).trim();
          if ("TUNING".equalsIgnoreCase(rawMode)) {
            parsedMode = Mode.TUNING;
          }
          headerSection = 0;
        } else if (trimmed.equals("# RuntimeOverrides:")) {
          headerSection = 1;
        } else if (trimmed.equals("# TableCellOverrides:")) {
          headerSection = 2;
        } else if (headerSection == 1 && trimmed.startsWith("#")) {
          String overrideLine = trimmed.substring(1).trim();
          int colonIdx = overrideLine.indexOf(": ");
          int arrowIdx = overrideLine.lastIndexOf(" -> ");
          if (colonIdx > 0 && arrowIdx > colonIdx) {
            String profAndKey = overrideLine.substring(0, colonIdx).trim();
            String newVal = overrideLine.substring(arrowIdx + 4).trim();
            int dotIdx = profAndKey.indexOf('.');
            if (dotIdx > 0) {
              String prof = profAndKey.substring(0, dotIdx).trim();
              String key = profAndKey.substring(dotIdx + 1).trim();
              parsedOverrides.put(new OverrideKey(normalizeProfileHeader(prof), key), newVal);
            }
          }
        } else if (headerSection == 2 && trimmed.startsWith("#")) {
          String overrideLine = trimmed.substring(1).trim();
          Matcher m = TABLE_OVERRIDE_HEADER_PATTERN.matcher(overrideLine);
          if (m.matches()) {
            String table = stripCsvExtension(m.group(1).trim()).toLowerCase(Locale.ROOT);
            String prof = normalizeProfileHeader(m.group(2).trim());
            String rowKey = m.group(3).trim();
            String col = m.group(4).trim();
            String newVal = m.group(6).trim();
            parsedTableOverrides.put(new TableCellOverrideKey(table, prof, rowKey, col), newVal);
          }
        }
      } else {
        fileSections.get(currentFile).append(line).append("\n");
      }
    }

    if (fileSections.isEmpty()) {
      throw new IllegalArgumentException(
          "No '=== <file>.csv ===' table sections found in effective snapshot");
    }

    Map<String, String> csvMap = new LinkedHashMap<>();
    for (Map.Entry<String, StringBuilder> e : fileSections.entrySet()) {
      csvMap.put(e.getKey(), e.getValue().toString());
    }
    FluxFig fig = FluxFig.fromCsvMap(csvMap, parsedActiveProfile);
    if (parsedMode == Mode.TUNING) {
      fig.setMode(Mode.TUNING);
    }
    for (Map.Entry<OverrideKey, String> ov : parsedOverrides.entrySet()) {
      fig.setOverrideUnchecked(ov.getKey().profile(), ov.getKey().key(), ov.getValue());
    }
    for (Map.Entry<TableCellOverrideKey, String> tov : parsedTableOverrides.entrySet()) {
      TableCellOverrideKey k = tov.getKey();
      fig.setTableCellOverrideUnchecked(
          k.tableName(), k.profile(), k.rowKey(), k.column(), tov.getValue());
    }
    return fig;
  }

  static void writeCsvFileAtomically(Path dir, String fileName, String content) throws IOException {
    if (fileName == null
        || fileName.isBlank()
        || fileName.contains("/")
        || fileName.contains("\\")
        || fileName.contains("..")) {
      throw new IllegalArgumentException("Refusing invalid CSV file name: " + fileName);
    }
    Path normDir = dir.toAbsolutePath().normalize();
    Path targetPath = normDir.resolve(fileName).normalize();
    if (!targetPath.startsWith(normDir)) {
      throw new IllegalArgumentException("CSV file path escapes config directory: " + fileName);
    }
    Path tmpPath = normDir.resolve("." + fileName + ".tmp");
    Files.writeString(tmpPath, normalizeLineEndings(content), StandardCharsets.UTF_8);
    Files.move(tmpPath, targetPath, StandardCopyOption.REPLACE_EXISTING);
  }

  static String computeConfigSha256(Map<String, String> normalizedFiles) {
    try {
      MessageDigest md = MessageDigest.getInstance("SHA-256");
      List<String> sortedFileNames = new ArrayList<>(normalizedFiles.keySet());
      sortedFileNames.sort(Comparator.naturalOrder());
      for (String fileName : sortedFileNames) {
        String tableName = stripCsvExtension(fileName);
        if (!"releases".equalsIgnoreCase(tableName)) {
          md.update((fileName + "\n").getBytes(StandardCharsets.UTF_8));
          md.update(normalizedFiles.get(fileName).getBytes(StandardCharsets.UTF_8));
        }
      }
      byte[] digest = md.digest();
      StringBuilder hex = new StringBuilder();
      for (int i = 0; i < 4; i++) {
        hex.append(String.format("%02x", digest[i]));
      }
      return hex.toString();
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("SHA-256 unavailable", e);
    }
  }

  static String formatActiveProfileDirective(String profile, Mode mode, String configSha256) {
    String normProf = normalizeProfileHeader(profile == null ? "DEFAULT" : profile);
    Mode effMode = mode == null ? Mode.PRODUCTION : mode;
    String sha = configSha256 == null ? "" : configSha256.trim();
    return normProf + "\nmode=" + effMode.name() + "\nConfigSHA256=" + sha + "\n";
  }

  static ActiveProfileDirective parseActiveProfileDirective(String rawText) {
    if (rawText == null || rawText.isBlank()) {
      return new ActiveProfileDirective("DEFAULT", Mode.PRODUCTION, "");
    }
    String profile = "DEFAULT";
    Mode parsedMode = Mode.PRODUCTION;
    String sha = "";
    boolean profileSet = false;
    for (String rawLine : normalizeLineEndings(rawText).lines().toList()) {
      String line = rawLine.trim();
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      int eq = line.indexOf('=');
      if (eq > 0) {
        String k = line.substring(0, eq).trim();
        String v = line.substring(eq + 1).trim();
        if ("profile".equalsIgnoreCase(k)) {
          profile = normalizeProfileHeader(v);
          profileSet = true;
        } else if ("mode".equalsIgnoreCase(k)) {
          parsedMode = "TUNING".equalsIgnoreCase(v) ? Mode.TUNING : Mode.PRODUCTION;
        } else if ("ConfigSHA256".equalsIgnoreCase(k)) {
          sha = v;
        }
      } else if (!profileSet) {
        profile = normalizeProfileHeader(line);
        profileSet = true;
      }
    }
    return new ActiveProfileDirective(profile, parsedMode, sha);
  }

  static void parseAndClassifyCsv(
      String tableName,
      String content,
      Map<String, List<String>> parameterHeaders,
      Map<String, String> parameterCanonicalKeys,
      Map<String, String> parameterKeySourceFile,
      Map<String, Map<String, String>> parameterTable,
      Map<String, List<Map<String, String>>> relationalTables,
      List<String> csvParseErrors) {
    List<String> lines = content.lines().toList();
    String[] headers = null;
    int headerLineIdx = -1;
    for (int i = 0; i < lines.size(); i++) {
      String trimmed = lines.get(i).trim();
      if (!trimmed.isEmpty() && !trimmed.startsWith("#")) {
        headers = parseCsvLine(trimmed);
        headerLineIdx = i;
        break;
      }
    }
    if (headers == null || headers.length == 0) {
      return;
    }

    String firstHeader = normalizeProfileHeader(headers[0]);
    if ("key".equalsIgnoreCase(firstHeader)) {
      List<String> normHeaders = new ArrayList<>();
      for (int c = 1; c < headers.length; c++) {
        normHeaders.add(normalizeProfileHeader(headers[c]));
      }
      String tableLower = tableName.toLowerCase(Locale.ROOT);
      parameterHeaders.put(tableLower, normHeaders);

      Set<String> seenNormKeysInFile = new LinkedHashSet<>();
      int dataRowNum = 0;
      for (int r = headerLineIdx + 1; r < lines.size(); r++) {
        String line = lines.get(r).trim();
        if (line.isEmpty() || line.startsWith("#")) {
          continue;
        }
        dataRowNum++;
        String[] cols = parseCsvLine(line);
        if (cols.length == 0 || cols[0].isEmpty()) {
          continue;
        }
        String key = cols[0];
        String normKey = normalizeIdentifier(key);
        if (!seenNormKeysInFile.add(normKey)) {
          csvParseErrors.add(
              tableName
                  + ".csv [row "
                  + dataRowNum
                  + ", col 'key']: duplicate parameter key '"
                  + key
                  + "'");
        } else if (parameterCanonicalKeys.containsKey(normKey)
            && !parameterKeySourceFile
                .get(parameterCanonicalKeys.get(normKey))
                .equalsIgnoreCase(tableName + ".csv")) {
          String existingKey = parameterCanonicalKeys.get(normKey);
          csvParseErrors.add(
              tableName
                  + ".csv [row "
                  + dataRowNum
                  + ", col 'key']: duplicate parameter key '"
                  + key
                  + "' already defined in "
                  + parameterKeySourceFile.get(existingKey));
        } else {
          parameterCanonicalKeys.put(normKey, key);
          parameterKeySourceFile.put(key, tableName + ".csv");
        }
        String canonicalKey = parameterCanonicalKeys.getOrDefault(normKey, key);
        Map<String, String> rowMap =
            parameterTable.computeIfAbsent(canonicalKey, k -> new LinkedHashMap<>());
        for (int c = 1; c < headers.length && c < cols.length; c++) {
          String val = cols[c];
          if (!val.isEmpty()) {
            rowMap.put(normalizeProfileHeader(headers[c]), val);
          }
        }
      }
    } else {
      List<Map<String, String>> rows = new ArrayList<>();
      for (int r = headerLineIdx + 1; r < lines.size(); r++) {
        String line = lines.get(r).trim();
        if (line.isEmpty() || line.startsWith("#")) {
          continue;
        }
        String[] cols = parseCsvLine(line);
        Map<String, String> row = new LinkedHashMap<>();
        for (int c = 0; c < headers.length; c++) {
          String cell = c < cols.length ? cols[c] : "";
          row.put(headers[c], cell);
        }
        rows.add(Collections.unmodifiableMap(row));
      }
      relationalTables.put(
          tableName.toLowerCase(Locale.ROOT), Collections.unmodifiableList(rows));
    }
  }

  static String computeNextReleaseVersionTag(List<ReleaseEntry> releases) {
    int maxV = 0;
    for (ReleaseEntry r : releases) {
      String v = r.version().trim();
      if ((v.startsWith("v") || v.startsWith("V")) && v.length() > 1) {
        try {
          int num = Integer.parseInt(v.substring(1));
          if (num > maxV) {
            maxV = num;
          }
        } catch (NumberFormatException ignored) {
        }
      }
    }
    return "v" + (maxV + 1);
  }

  static String serializeReleaseEntryCsv(
      String rawReleasesCsv, List<Map<String, String>> existingRows, ReleaseEntry newEntry) {
    List<String> leadingComments = extractLeadingCommentLines(rawReleasesCsv);
    List<String> headers =
        extractHeaderColumns(
            rawReleasesCsv, List.of("version", "configSha256", "timestamp", "author", "notes"));
    List<Map<String, String>> rows = new ArrayList<>(existingRows);
    Map<String, String> newRow = new LinkedHashMap<>();
    putCellPreservingHeaderCase(newRow, headers, "version", newEntry.version());
    putCellPreservingHeaderCase(newRow, headers, "configSha256", newEntry.configSha256());
    putCellPreservingHeaderCase(newRow, headers, "timestamp", newEntry.timestamp());
    putCellPreservingHeaderCase(newRow, headers, "author", newEntry.author());
    putCellPreservingHeaderCase(newRow, headers, "notes", newEntry.notes());
    rows.add(newRow);
    return serializeRelationalTable(leadingComments, headers, rows);
  }

  static String formatDiffSummary(Map<String, String> diffs) {
    if (diffs.isEmpty()) {
      return "";
    }
    StringBuilder sb = new StringBuilder();
    int i = 0;
    for (Map.Entry<String, String> e : diffs.entrySet()) {
      if (i++ > 0) {
        sb.append("; ");
      }
      sb.append(e.getKey()).append(": ").append(e.getValue());
    }
    return sb.toString();
  }

  static Map<String, String> buildCandidateFilesWithOverridesApplied(
      FluxFig fig,
      Map<String, String> rawCsvFiles,
      Map<OverrideKey, String> runtimeOverrides,
      Map<TableCellOverrideKey, String> tableCellOverrides,
      Map<String, List<String>> parameterHeaders,
      Map<String, Map<String, String>> parameterTable,
      Map<String, String> parameterKeySourceFile) {
    Map<String, String> candidateFiles = new LinkedHashMap<>(rawCsvFiles);

    if (!runtimeOverrides.isEmpty()) {
      Set<String> affectedFiles = new LinkedHashSet<>();
      for (OverrideKey ok : runtimeOverrides.keySet()) {
        String key = fig.resolveCanonicalParameterKey(ok.key());
        affectedFiles.add(parameterKeySourceFile.getOrDefault(key, "parameters.csv"));
      }

      for (String targetFile : affectedFiles) {
        String rawParams = candidateFiles.get(targetFile);
        List<String> leadingComments = extractLeadingCommentLines(rawParams);
        List<String> rawHeaders = extractHeaderColumns(rawParams, List.of("key", "DEFAULT"));
        String keyHeader = rawHeaders.isEmpty() ? "key" : rawHeaders.get(0);
        List<String> profileHeaders = new ArrayList<>();
        List<String> normProfiles = new ArrayList<>();
        for (int i = 1; i < rawHeaders.size(); i++) {
          profileHeaders.add(rawHeaders.get(i));
          normProfiles.add(normalizeProfileHeader(rawHeaders.get(i)));
        }
        String tableStem =
            targetFile.toLowerCase(Locale.ROOT).endsWith(".csv")
                ? targetFile.substring(0, targetFile.length() - 4)
                : targetFile;
        if (profileHeaders.isEmpty()) {
          for (String p :
              parameterHeaders.getOrDefault(tableStem, new ArrayList<>(List.of("DEFAULT")))) {
            profileHeaders.add(p);
            normProfiles.add(normalizeProfileHeader(p));
          }
        }

        Map<String, Map<String, String>> updatedParams = new LinkedHashMap<>();
        for (Map.Entry<String, Map<String, String>> e : parameterTable.entrySet()) {
          String srcFile = parameterKeySourceFile.getOrDefault(e.getKey(), "parameters.csv");
          if (targetFile.equalsIgnoreCase(srcFile)) {
            updatedParams.put(e.getKey(), new LinkedHashMap<>(e.getValue()));
          }
        }
        for (Map.Entry<OverrideKey, String> ov : runtimeOverrides.entrySet()) {
          String prof = ov.getKey().profile();
          String key = fig.resolveCanonicalParameterKey(ov.getKey().key());
          String srcFile = parameterKeySourceFile.getOrDefault(key, "parameters.csv");
          if (!targetFile.equalsIgnoreCase(srcFile)) {
            continue;
          }
          if (!normProfiles.contains(prof)) {
            profileHeaders.add(prof);
            normProfiles.add(prof);
          }
          updatedParams.computeIfAbsent(key, k -> new LinkedHashMap<>()).put(prof, ov.getValue());
        }

        StringBuilder sb = new StringBuilder();
        for (String cmt : leadingComments) {
          sb.append(cmt).append('\n');
        }
        sb.append(formatCsvCell(keyHeader));
        for (String ph : profileHeaders) {
          sb.append(',').append(formatCsvCell(ph));
        }
        sb.append('\n');
        for (Map.Entry<String, Map<String, String>> rowEntry : updatedParams.entrySet()) {
          sb.append(formatCsvCell(rowEntry.getKey()));
          Map<String, String> cells = rowEntry.getValue();
          for (String np : normProfiles) {
            sb.append(',').append(formatCsvCell(cells.getOrDefault(np, "")));
          }
          sb.append('\n');
        }
        candidateFiles.put(targetFile, normalizeLineEndings(sb.toString()));
      }
    }

    if (!tableCellOverrides.isEmpty()) {
      Set<String> affectedTables = new LinkedHashSet<>();
      for (TableCellOverrideKey tk : tableCellOverrides.keySet()) {
        affectedTables.add(tk.tableName());
      }
      for (String tableName : affectedTables) {
        String fileName = tableName + ".csv";
        String rawTableCsv = candidateFiles.get(fileName);
        List<String> leadingComments = extractLeadingCommentLines(rawTableCsv);
        List<Map<String, String>> updatedRows = fig.getTableWithCellOverrides(tableName);
        List<String> headers =
            extractHeaderColumns(
                rawTableCsv,
                updatedRows.isEmpty()
                    ? List.of()
                    : new ArrayList<>(updatedRows.get(0).keySet()));
        if (!headers.isEmpty()) {
          String serialized = serializeRelationalTable(leadingComments, headers, updatedRows);
          candidateFiles.put(fileName, normalizeLineEndings(serialized));
        }
      }
    }

    return candidateFiles;
  }

  static String formatEffectiveSnapshot(
      FluxFig fig,
      Mode mode,
      String effectiveVersion,
      String configSha256,
      String activeProfile,
      Map<OverrideKey, String> runtimeOverrides,
      Map<TableCellOverrideKey, String> tableCellOverrides,
      Map<String, String> rawCsvFiles) {
    StringBuilder sb = new StringBuilder();
    sb.append("# FluxFig Effective Snapshot\n");
    sb.append("# Mode: ").append(mode.name()).append("\n");
    sb.append("# EffectiveVersion: ").append(effectiveVersion).append("\n");
    sb.append("# ConfigSHA256: ").append(configSha256).append("\n");
    sb.append("# ActiveProfile: ").append(activeProfile).append("\n");

    Map<String, String> paramDiffs = new LinkedHashMap<>();
    for (Map.Entry<OverrideKey, String> entry : runtimeOverrides.entrySet()) {
      String prof = entry.getKey().profile();
      String key = entry.getKey().key();
      String baseVal = fig.resolveFromCsvOnly(prof, key).orElse("<unset>");
      if (!baseVal.equals(entry.getValue())) {
        paramDiffs.put(prof + "." + key, baseVal + " -> " + entry.getValue());
      }
    }
    if (!paramDiffs.isEmpty()) {
      sb.append("# RuntimeOverrides:\n");
      for (Map.Entry<String, String> d : paramDiffs.entrySet()) {
        sb.append("#   ").append(d.getKey()).append(": ").append(d.getValue()).append("\n");
      }
    }

    Map<String, String> tableDiffs = new LinkedHashMap<>();
    for (Map.Entry<TableCellOverrideKey, String> entry : tableCellOverrides.entrySet()) {
      TableCellOverrideKey tk = entry.getKey();
      String baseVal =
          fig.resolveTableCellFromCsvOnly(tk.tableName(), tk.profile(), tk.rowKey(), tk.column())
              .orElse("<unset>");
      if (!baseVal.equals(entry.getValue())) {
        tableDiffs.put(
            tk.tableName() + "[" + tk.profile() + ":" + tk.rowKey() + "]." + tk.column(),
            baseVal + " -> " + entry.getValue());
      }
    }
    if (!tableDiffs.isEmpty()) {
      sb.append("# TableCellOverrides:\n");
      for (Map.Entry<String, String> d : tableDiffs.entrySet()) {
        sb.append("#   ").append(d.getKey()).append(": ").append(d.getValue()).append("\n");
      }
    }

    for (Map.Entry<String, String> fileEntry : rawCsvFiles.entrySet()) {
      sb.append("\n=== ").append(fileEntry.getKey()).append(" ===\n");
      sb.append(fileEntry.getValue().stripTrailing()).append("\n");
    }
    return sb.toString();
  }
}
