package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.DefaultValue;
import frc.lib.fluxfig.FluxFig.RowKey;
import frc.lib.fluxfig.FluxFig.TableCellOverrideKey;
import java.lang.reflect.Constructor;
import java.lang.reflect.RecordComponent;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

/**
 * Package-private reflective Java {@code record} binder, {@link RowKey} preset lookup resolver, and
 * continuous piecewise linear table interpolator for {@link FluxFig}.
 */
final class FluxFigRecordBinder {

  private static final List<String> CANDIDATE_KEY_COLUMNS =
      List.of(
          "preset",
          "presetname",
          "mechanism",
          "cameraname",
          "batteryname",
          "version",
          "name",
          "id",
          "distancemeters");

  private FluxFigRecordBinder() {}

  static String findProfileColumnHeader(Map<String, String> sampleRow) {
    for (String header : sampleRow.keySet()) {
      String clean = FluxFigCsvCodec.normalizeProfileHeader(header);
      if ("robot".equalsIgnoreCase(clean) || "profile".equalsIgnoreCase(clean)) {
        return header;
      }
    }
    return null;
  }

  static String findRowKeyColumnHeader(
      Map<String, String> sampleRow, Class<?> optionalRecordClass) {
    if (optionalRecordClass != null && optionalRecordClass.isRecord()) {
      for (RecordComponent rc : optionalRecordClass.getRecordComponents()) {
        if (rc.getAnnotation(RowKey.class) != null) {
          for (String h : sampleRow.keySet()) {
            if (FluxFigCsvCodec.normalizeIdentifier(h)
                .equals(FluxFigCsvCodec.normalizeIdentifier(rc.getName()))) {
              return h;
            }
          }
        }
      }
    }
    for (String candidate : CANDIDATE_KEY_COLUMNS) {
      for (String header : sampleRow.keySet()) {
        if (FluxFigCsvCodec.normalizeIdentifier(header).equals(candidate)) {
          return header;
        }
      }
    }
    for (String header : sampleRow.keySet()) {
      String clean = FluxFigCsvCodec.normalizeProfileHeader(header);
      if (!"robot".equalsIgnoreCase(clean) && !"profile".equalsIgnoreCase(clean)) {
        return header;
      }
    }
    return null;
  }

  static RecordComponent findKeyRecordComponent(Class<?> recordClass) {
    RecordComponent[] comps = recordClass.getRecordComponents();
    for (RecordComponent rc : comps) {
      if (rc.getAnnotation(RowKey.class) != null) {
        return rc;
      }
    }
    List<String> commonKeys =
        List.of("preset", "presetname", "mechanism", "cameraname", "batteryname", "name", "id");
    for (String candidate : commonKeys) {
      for (RecordComponent rc : comps) {
        if (FluxFigCsvCodec.normalizeIdentifier(rc.getName()).equals(candidate)) {
          return rc;
        }
      }
    }
    for (RecordComponent rc : comps) {
      String norm = FluxFigCsvCodec.normalizeIdentifier(rc.getName());
      if (!"robot".equals(norm) && !"profile".equals(norm)) {
        return rc;
      }
    }
    return comps.length > 0 ? comps[0] : null;
  }

  static <T extends Record> List<T> bindRowsToRecord(
      String tableLabel,
      List<Map<String, String>> rows,
      Class<T> recordClass,
      boolean lenient,
      List<String> parseErrorsOut) {
    if (rows.isEmpty()) {
      return Collections.emptyList();
    }
    RecordComponent[] components = recordClass.getRecordComponents();
    Class<?>[] paramTypes = new Class<?>[components.length];
    DefaultValue[] defaultAnnotations = new DefaultValue[components.length];
    for (int i = 0; i < components.length; i++) {
      paramTypes[i] = components[i].getType();
      defaultAnnotations[i] = components[i].getAnnotation(DefaultValue.class);
    }
    try {
      Constructor<T> ctor = recordClass.getDeclaredConstructor(paramTypes);
      ctor.setAccessible(true);
      List<T> result = new ArrayList<>(rows.size());
      for (int r = 0; r < rows.size(); r++) {
        Map<String, String> row = rows.get(r);
        int rowNum = r + 1;
        try {
          Object[] args = new Object[components.length];
          for (int i = 0; i < components.length; i++) {
            String compName = components[i].getName();
            String rawCell = FluxFigCsvCodec.findCellByColumnName(row, compName);
            DefaultValue defAnn = defaultAnnotations[i];
            if (rawCell == null || (rawCell.trim().isEmpty() && defAnn != null)) {
              if (defAnn != null) {
                rawCell = defAnn.value();
              } else {
                throw new IllegalArgumentException(
                    tableLabel
                        + " [row "
                        + rowNum
                        + ", col '"
                        + compName
                        + "']: missing column for record "
                        + recordClass.getSimpleName()
                        + " in row headers "
                        + row.keySet());
              }
            } else if (rawCell.trim().isEmpty()
                && (paramTypes[i].isPrimitive() || paramTypes[i] == String.class)
                && !"notes".equalsIgnoreCase(compName)
                && !"qrcode".equalsIgnoreCase(compName)) {
              throw new IllegalArgumentException(
                  tableLabel
                      + " [row "
                      + rowNum
                      + ", col '"
                      + compName
                      + "']: missing or empty required value");
            }
            try {
              args[i] = convertCell(rawCell, paramTypes[i]);
            } catch (IllegalArgumentException cellErr) {
              throw new IllegalArgumentException(
                  tableLabel
                      + " [row "
                      + rowNum
                      + ", col '"
                      + compName
                      + "']: "
                      + formatCellErrorMessage(rawCell.trim(), paramTypes[i], cellErr),
                  cellErr);
            }
          }
          result.add(ctor.newInstance(args));
        } catch (IllegalArgumentException rowErr) {
          if (lenient && parseErrorsOut != null) {
            parseErrorsOut.add(rowErr.getMessage());
          } else {
            throw rowErr;
          }
        }
      }
      return Collections.unmodifiableList(result);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(
          "Failed to bind table rows to record " + recordClass.getName(), e);
    }
  }

  static <T extends Record> Optional<T> findRowByKey(
      List<T> rows, Object rowKey, Class<T> recordClass) {
    if (rows.isEmpty() || rowKey == null) {
      return Optional.empty();
    }
    RecordComponent keyComp = findKeyRecordComponent(recordClass);
    if (keyComp == null) {
      return Optional.empty();
    }
    String targetNorm = FluxFigCsvCodec.normalizeIdentifier(String.valueOf(rowKey));
    try {
      for (T row : rows) {
        Object val = keyComp.getAccessor().invoke(row);
        if (val != null
            && FluxFigCsvCodec.normalizeIdentifier(String.valueOf(val)).equals(targetNorm)) {
          return Optional.of(row);
        }
      }
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Failed to read row key on " + recordClass.getName(), e);
    }
    return Optional.empty();
  }

  static <T extends Record> T interpolateRows(
      String tableName,
      String profile,
      List<T> boundRows,
      String xColumn,
      double xValue,
      Class<T> recordClass) {
    List<T> rows = new ArrayList<>(boundRows);
    if (rows.isEmpty()) {
      throw new IllegalArgumentException(
          "Cannot interpolate empty table '" + tableName + "' for profile '" + profile + "'");
    }
    RecordComponent[] components = recordClass.getRecordComponents();
    int xCompIdx = -1;
    String normX = FluxFigCsvCodec.normalizeIdentifier(xColumn);
    for (int i = 0; i < components.length; i++) {
      if (FluxFigCsvCodec.normalizeIdentifier(components[i].getName()).equals(normX)) {
        xCompIdx = i;
        break;
      }
    }
    if (xCompIdx < 0) {
      throw new IllegalArgumentException(
          "Interpolation column '"
              + xColumn
              + "' not found on record "
              + recordClass.getSimpleName());
    }
    final RecordComponent xComp = components[xCompIdx];
    rows.sort(Comparator.comparingDouble(r -> readNumericComponent(r, xComp)));

    double firstX = readNumericComponent(rows.get(0), xComp);
    if (xValue <= firstX || rows.size() == 1) {
      return rows.get(0);
    }
    double lastX = readNumericComponent(rows.get(rows.size() - 1), xComp);
    if (xValue >= lastX) {
      return rows.get(rows.size() - 1);
    }

    int lowerIdx = 0;
    for (int i = 0; i < rows.size() - 1; i++) {
      double x0 = readNumericComponent(rows.get(i), xComp);
      double x1 = readNumericComponent(rows.get(i + 1), xComp);
      if (xValue >= x0 && xValue <= x1) {
        lowerIdx = i;
        break;
      }
    }

    T lowRow = rows.get(lowerIdx);
    T highRow = rows.get(lowerIdx + 1);
    double x0 = readNumericComponent(lowRow, xComp);
    double x1 = readNumericComponent(highRow, xComp);
    double t = (Math.abs(x1 - x0) < 1e-9) ? 0.0 : (xValue - x0) / (x1 - x0);

    Class<?>[] paramTypes = new Class<?>[components.length];
    Object[] interpolatedArgs = new Object[components.length];
    try {
      for (int i = 0; i < components.length; i++) {
        RecordComponent rc = components[i];
        Class<?> type = rc.getType();
        paramTypes[i] = type;
        if (i == xCompIdx) {
          interpolatedArgs[i] = castNumeric(xValue, type);
        } else if (isNumericType(type)) {
          double v0 = readNumericComponent(lowRow, rc);
          double v1 = readNumericComponent(highRow, rc);
          double lerped = v0 + t * (v1 - v0);
          interpolatedArgs[i] = castNumeric(lerped, type);
        } else {
          T nearest = (t < 0.5) ? lowRow : highRow;
          interpolatedArgs[i] = rc.getAccessor().invoke(nearest);
        }
      }
      Constructor<T> ctor = recordClass.getDeclaredConstructor(paramTypes);
      ctor.setAccessible(true);
      return ctor.newInstance(interpolatedArgs);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(
          "Failed to construct interpolated record " + recordClass.getName(), e);
    }
  }

  private static String formatCellErrorMessage(
      String trimmedValue, Class<?> targetType, IllegalArgumentException cause) {
    if (targetType == double.class
        || targetType == Double.class
        || targetType == float.class
        || targetType == Float.class) {
      return "invalid number '" + trimmedValue + "'";
    }
    if (targetType == int.class
        || targetType == Integer.class
        || targetType == long.class
        || targetType == Long.class) {
      return "invalid integer '" + trimmedValue + "'";
    }
    if (targetType.isEnum()) {
      return cause.getMessage();
    }
    return "invalid " + targetType.getSimpleName() + " value '" + trimmedValue + "'";
  }

  @SuppressWarnings({"unchecked", "rawtypes"})
  private static Object convertCell(String raw, Class<?> targetType) {
    String trimmed = raw.trim();
    if (targetType == String.class) {
      return trimmed;
    }
    if (targetType == int.class || targetType == Integer.class) {
      return trimmed.isEmpty() ? 0 : Integer.parseInt(trimmed);
    }
    if (targetType == long.class || targetType == Long.class) {
      return trimmed.isEmpty() ? 0L : Long.parseLong(trimmed);
    }
    if (targetType == double.class || targetType == Double.class) {
      return trimmed.isEmpty() ? 0.0 : Double.parseDouble(trimmed);
    }
    if (targetType == float.class || targetType == Float.class) {
      return trimmed.isEmpty() ? 0.0f : Float.parseFloat(trimmed);
    }
    if (targetType == boolean.class || targetType == Boolean.class) {
      return !trimmed.isEmpty() && Boolean.parseBoolean(trimmed);
    }
    if (targetType.isEnum()) {
      Class<Enum> enumClass = (Class<Enum>) targetType;
      try {
        return Enum.valueOf(enumClass, trimmed);
      } catch (IllegalArgumentException ignored) {
        String norm = FluxFigCsvCodec.normalizeIdentifier(trimmed);
        for (Enum constant : enumClass.getEnumConstants()) {
          if (FluxFigCsvCodec.normalizeIdentifier(constant.name()).equals(norm)) {
            return constant;
          }
        }
        throw new IllegalArgumentException(
            "Unknown enum constant '" + trimmed + "' for " + targetType.getSimpleName());
      }
    }
    throw new IllegalArgumentException("Unsupported record field type: " + targetType.getName());
  }

  private static boolean isNumericType(Class<?> type) {
    return type == double.class
        || type == Double.class
        || type == float.class
        || type == Float.class
        || type == int.class
        || type == Integer.class
        || type == long.class
        || type == Long.class;
  }

  private static double readNumericComponent(Object recordInstance, RecordComponent rc) {
    try {
      Object v = rc.getAccessor().invoke(recordInstance);
      if (v instanceof Number n) {
        return n.doubleValue();
      }
      throw new IllegalArgumentException("Component '" + rc.getName() + "' is not numeric: " + v);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Failed to read component " + rc.getName(), e);
    }
  }

  private static Object castNumeric(double val, Class<?> type) {
    if (type == double.class || type == Double.class) {
      return val;
    }
    if (type == float.class || type == Float.class) {
      return (float) val;
    }
    if (type == int.class || type == Integer.class) {
      return (int) Math.round(val);
    }
    if (type == long.class || type == Long.class) {
      return Math.round(val);
    }
    return val;
  }

  static Optional<String> resolveTableCellFromCsvOnly(
      Map<String, List<Map<String, String>>> relationalTables,
      String tableName,
      String profile,
      String rowKey,
      String column) {
    List<Map<String, String>> rawRows =
        relationalTables.getOrDefault(
            FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT),
            Collections.emptyList());
    if (rawRows.isEmpty()) {
      return Optional.empty();
    }
    String profileCol = findProfileColumnHeader(rawRows.get(0));
    String keyCol = findRowKeyColumnHeader(rawRows.get(0), null);
    if (keyCol == null) {
      return Optional.empty();
    }
    for (String candidateProfile : FluxFig.buildProfileHierarchy(profile)) {
      for (Map<String, String> row : rawRows) {
        if (profileCol != null) {
          String rowProf =
              FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, ""));
          if (!rowProf.isEmpty()
              && !"*".equals(rowProf)
              && !rowProf.equalsIgnoreCase(candidateProfile)) {
            continue;
          }
        }
        String cellKey = row.getOrDefault(keyCol, "");
        if (FluxFigCsvCodec.normalizeIdentifier(cellKey)
            .equals(FluxFigCsvCodec.normalizeIdentifier(rowKey))) {
          String v = FluxFigCsvCodec.findCellByColumnName(row, column);
          return Optional.ofNullable(v);
        }
      }
    }
    return Optional.empty();
  }

  static List<Map<String, String>> resolveTableWithCellOverrides(
      Map<String, List<Map<String, String>>> relationalTables,
      Map<TableCellOverrideKey, String> tableCellOverrides,
      String tableName) {
    String normalized = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    List<Map<String, String>> baseRows =
        relationalTables.getOrDefault(normalized, Collections.emptyList());
    if (baseRows.isEmpty() || tableCellOverrides.isEmpty()) {
      return baseRows;
    }
    String profileCol = findProfileColumnHeader(baseRows.get(0));
    String keyCol = findRowKeyColumnHeader(baseRows.get(0), null);
    if (keyCol == null) {
      return baseRows;
    }

    List<Map<String, String>> updated = new ArrayList<>(baseRows.size());
    for (Map<String, String> row : baseRows) {
      String rowProf =
          profileCol != null
              ? FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, ""))
              : "DEFAULT";
      String rowKey = row.getOrDefault(keyCol, "");
      Map<String, String> mutable = null;
      for (Map.Entry<TableCellOverrideKey, String> ov : tableCellOverrides.entrySet()) {
        TableCellOverrideKey tk = ov.getKey();
        if (!tk.tableName().equalsIgnoreCase(normalized)) {
          continue;
        }
        if (!FluxFigCsvCodec.normalizeIdentifier(tk.rowKey())
            .equals(FluxFigCsvCodec.normalizeIdentifier(rowKey))) {
          continue;
        }
        boolean profMatch =
            profileCol == null
                || rowProf.isEmpty()
                || "*".equals(rowProf)
                || FluxFig.buildProfileHierarchy(tk.profile()).stream()
                    .anyMatch(p -> p.equalsIgnoreCase(rowProf));
        if (profMatch) {
          if (mutable == null) {
            mutable = new LinkedHashMap<>(row);
          }
          FluxFigCsvCodec.putCellPreservingHeaderCase(
              mutable, new ArrayList<>(mutable.keySet()), tk.column(), ov.getValue());
        }
      }
      updated.add(mutable != null ? Collections.unmodifiableMap(mutable) : row);
    }
    return Collections.unmodifiableList(updated);
  }

  static List<Map<String, String>> resolveTableForProfile(
      Map<String, List<Map<String, String>>> relationalTables,
      Map<TableCellOverrideKey, String> tableCellOverrides,
      String tableName,
      String profile) {
    List<Map<String, String>> allRows =
        resolveTableWithCellOverrides(relationalTables, tableCellOverrides, tableName);
    if (allRows.isEmpty()) {
      return allRows;
    }
    String profileCol = findProfileColumnHeader(allRows.get(0));
    if (profileCol == null) {
      return allRows;
    }

    List<String> hierarchy = FluxFig.buildProfileHierarchy(profile);
    String keyCol = findRowKeyColumnHeader(allRows.get(0), null);

    List<String> reverseHierarchy = new ArrayList<>(hierarchy);
    Collections.reverse(reverseHierarchy);

    if (keyCol != null) {
      Map<String, Map<String, String>> mergedByRowKey = new LinkedHashMap<>();
      for (Map<String, String> row : allRows) {
        String cell = FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, ""));
        if (cell.isEmpty() || "*".equals(cell)) {
          String rk = FluxFigCsvCodec.normalizeIdentifier(row.getOrDefault(keyCol, ""));
          mergedByRowKey.put(rk, row);
        }
      }
      for (String candidateProf : reverseHierarchy) {
        for (Map<String, String> row : allRows) {
          String cell = FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, ""));
          if (cell.equalsIgnoreCase(candidateProf)) {
            String rk = FluxFigCsvCodec.normalizeIdentifier(row.getOrDefault(keyCol, ""));
            Map<String, String> effectiveRow =
                applyProfileOverlayCellOverrides(
                    tableCellOverrides, tableName, profile, row.getOrDefault(keyCol, ""), row);
            mergedByRowKey.put(rk, effectiveRow);
          }
        }
      }
      return Collections.unmodifiableList(new ArrayList<>(mergedByRowKey.values()));
    }

    for (String candidateProf : hierarchy) {
      List<Map<String, String>> levelRows = new ArrayList<>();
      for (Map<String, String> row : allRows) {
        String cell = FluxFigCsvCodec.normalizeProfileHeader(row.getOrDefault(profileCol, ""));
        if (cell.isEmpty() || "*".equals(cell) || cell.equalsIgnoreCase(candidateProf)) {
          levelRows.add(row);
        }
      }
      if (!levelRows.isEmpty()) {
        return Collections.unmodifiableList(levelRows);
      }
    }
    return Collections.emptyList();
  }

  private static Map<String, String> applyProfileOverlayCellOverrides(
      Map<TableCellOverrideKey, String> tableCellOverrides,
      String tableName,
      String requestedProfile,
      String rawRowKey,
      Map<String, String> baseRow) {
    if (tableCellOverrides.isEmpty()) {
      return baseRow;
    }
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    Map<String, String> mutable = null;
    List<String> reverseHierarchy = new ArrayList<>(FluxFig.buildProfileHierarchy(requestedProfile));
    Collections.reverse(reverseHierarchy);
    for (String profLevel : reverseHierarchy) {
      for (Map.Entry<TableCellOverrideKey, String> ov : tableCellOverrides.entrySet()) {
        TableCellOverrideKey tk = ov.getKey();
        if (tk.tableName().equalsIgnoreCase(cleanTable)
            && tk.profile().equalsIgnoreCase(profLevel)
            && FluxFigCsvCodec.normalizeIdentifier(tk.rowKey())
                .equals(FluxFigCsvCodec.normalizeIdentifier(rawRowKey))) {
          if (mutable == null) {
            mutable = new LinkedHashMap<>(baseRow);
          }
          FluxFigCsvCodec.putCellPreservingHeaderCase(
              mutable, new ArrayList<>(mutable.keySet()), tk.column(), ov.getValue());
        }
      }
    }
    return mutable != null ? Collections.unmodifiableMap(mutable) : baseRow;
  }
}
