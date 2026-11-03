package frc.lib.fluxfig;

import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.RecordComponent;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 * Pure-Java 17 standalone CSV and Google Sheets configuration engine, 1-cell controller fallback
 * resolver, schema-free relational table binder, live floor-tuning &amp; distance-table
 * interpolator, deterministic {@code ConfigSHA256} stamper, version harvester, and hardware safety
 * validator.
 *
 * <p>Zero WPILib dependencies and Java 17 bytecode compatibility ({@code --release 17}) so the
 * exact same {@code fluxfig.jar} works across both roboRIO (JDK 17) and SystemCore/laptop (JDK 25):
 *
 * <ul>
 *   <li>{@code Flux-SystemCore-2027} (WPILib 2027 / JDK 25)
 *   <li>{@code Flux-RoboRIO-2027} &amp; {@code New_Robot} (WPILib 2026 / JDK 17)
 *   <li>{@code FluxTray.jar} (Driver Station SFTP / mode-switcher / version harvester / JDK 25)
 *   <li>{@code FluxProcessor} (post-match telemetry &amp; battery analyzer / JDK 25)
 * </ul>
 */
public final class FluxFig {

  /** Default sheet config filename inside {@code configDir} for zero-arg Google Sheet sync. */
  public static final String SHEET_PROPERTIES_FILE = "fluxfig-sheet.properties";

  /**
   * Remote file inside {@code deploy/configs/} written by {@code FluxTray} to switch active profile
   * overlays and toggle {@link Mode#PRODUCTION} vs. {@link Mode#TUNING} in {@code < 50 ms} while
   * Disabled.
   */
  public static final String ACTIVE_PROFILE_FILE = "active-profile.txt";

  /**
   * Default RAM-disk snapshot path on the robot ({@code /tmp/fluxfig-tuned-snapshot.txt}) where
   * {@link Mode#TUNING} writes temporary live-tuned diffs on Disable so {@code FluxTray} can
   * harvest them over SFTP without mutating {@code deploy/configs/*.csv} on the robot.
   */
  public static final Path DEFAULT_TMP_TUNED_SNAPSHOT_PATH =
      Path.of("/tmp/fluxfig-tuned-snapshot.txt");

  /** Topic prefix used when publishing {@link Tunable} parameters and table cells in TUNING mode. */
  public static final String TUNING_TOPIC_PREFIX = "/FluxFig/Tuning/";

  /** Default core tabs fetched from Google Sheets when no explicit tab list or local CSVs exist. */
  public static final List<String> DEFAULT_SHEET_TABS =
      List.of("parameters", "mechanisms", "cameras", "batteries");

  private static final Set<String> STRUCTURAL_PARAMETER_KEYS =
      Set.of(
          "controllerprofile",
          "swervecanbus",
          "drivecanbus",
          "mechcanbus",
          "auxcanbus",
          "visionbackend",
          "coordinatesystem",
          "simulatemissingmechanisms",
          "drivestatorlimitamps",
          "drivesupplylimitamps",
          "driveslipcurrentamps");

  /**
   * Operating mode for {@link FluxFig}:
   *
   * <ul>
   *   <li>{@link #PRODUCTION}: Default competition/match mode. Tables and parameters are locked to
   *       validated CSV values; zero dashboard topics are published or polled. Automatically forced
   *       whenever FMS is attached.
   *   <li>{@link #TUNING}: Shop/practice floor tuning mode. Publishes {@link Tunable} parameters
   *       and relational table cells (e.g. {@code range_table/MID/shooterRps}) to {@link
   *       TuningBackend}, polls on-the-fly changes, updates in-memory presets and interpolation
   *       tables, and writes {@code /tmp/fluxfig-tuned-snapshot.txt} for {@code FluxTray} to
   *       collect and promote into a new versioned CSV release on the laptop.
   * </ul>
   */
  public enum Mode {
    PRODUCTION,
    TUNING
  }

  /**
   * Optional default value annotation for Java {@code record} components bound via {@link
   * #bindTable(String, Class)}. When a CSV column is omitted or blank, the annotated default string
   * is parsed instead of failing with a missing-column error.
   */
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.RECORD_COMPONENT, ElementType.PARAMETER})
  public @interface DefaultValue {
    String value();
  }

  /**
   * Marks a Java {@code record} component in a relational table (such as {@code shooterRps}, {@code
   * hoodRot}, or {@code feederRps} in {@code range_table.csv}, or {@code carriageMeters} in {@code
   * elevator_presets.csv}) as dashboard-tunable when {@link FluxFig} runs in {@link Mode#TUNING}.
   */
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.RECORD_COMPONENT, ElementType.PARAMETER})
  public @interface Tunable {}

  /**
   * Marks the Java {@code record} component that uniquely identifies a row within a robot profile
   * (such as {@code @RowKey ShotPreset preset} in {@code range_table.csv} or {@code @RowKey String
   * presetName} in {@code elevator_presets.csv}).
   */
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.RECORD_COMPONENT, ElementType.PARAMETER})
  public @interface RowKey {}

  /**
   * Pure-Java callback interface bridging {@link FluxFig} in {@link Mode#TUNING} to NetworkTables /
   * Elastic on the robot (or an in-memory map in unit tests) without introducing any WPILib
   * dependency into {@code fluxfig.jar}.
   */
  public interface TuningBackend {
    void publish(String topic, String value);

    Optional<String> read(String topic);
  }

  /**
   * Built-in thread-safe in-memory {@link TuningBackend} for unit tests, CLI simulations, and
   * desktop test harnesses.
   */
  public static final class InMemoryTuningBackend implements TuningBackend {
    private final Map<String, String> topics = new LinkedHashMap<>();

    @Override
    public synchronized void publish(String topic, String value) {
      topics.put(topic, value);
    }

    @Override
    public synchronized Optional<String> read(String topic) {
      return Optional.ofNullable(topics.get(topic));
    }

    public synchronized void putFromDashboard(String topic, Object newValue) {
      String fullTopic =
          topic.startsWith(TUNING_TOPIC_PREFIX) ? topic : TUNING_TOPIC_PREFIX + topic;
      topics.put(fullTopic, String.valueOf(newValue));
    }

    public synchronized Map<String, String> getPublishedTopics() {
      return Collections.unmodifiableMap(new LinkedHashMap<>(topics));
    }
  }

  /** Functional hook for fetching a single CSV tab from a Google Sheet (or test server). */
  @FunctionalInterface
  public interface SheetTabFetcher {
    String fetchCsvTab(String sheetIdOrUrl, String tabName) throws IOException;
  }

  /** Parsed contents of {@code deploy/configs/active-profile.txt} written by {@code FluxTray}. */
  public record ActiveProfileDirective(String profile, Mode mode, String expectedConfigSha256) {}

  public record MechanismSpec(
      String robot,
      @RowKey String mechanism,
      int motorId,
      String canBus,
      double statorLimitAmps,
      double supplyLimitAmps,
      double slipCurrentAmps,
      String gravityType,
      @Tunable double kP,
      @Tunable double kI,
      @Tunable double kD,
      @Tunable double kS,
      @Tunable double kV,
      @Tunable double kA,
      @Tunable double kG,
      @Tunable double minPosRot,
      @Tunable double maxPosRot,
      @DefaultValue("-1") int followerMotorId,
      @DefaultValue("false") boolean invertFollower,
      @DefaultValue("false") boolean motorInverted,
      @DefaultValue("1.0") double gearRatio) {

    public MechanismSpec(
        String robot,
        String mechanism,
        int motorId,
        String canBus,
        double statorLimitAmps,
        double supplyLimitAmps,
        double slipCurrentAmps,
        String gravityType,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        double minPosRot,
        double maxPosRot,
        int followerMotorId,
        boolean invertFollower) {
      this(
          robot,
          mechanism,
          motorId,
          canBus,
          statorLimitAmps,
          supplyLimitAmps,
          slipCurrentAmps,
          gravityType,
          kP,
          kI,
          kD,
          kS,
          kV,
          kA,
          kG,
          minPosRot,
          maxPosRot,
          followerMotorId,
          invertFollower,
          false,
          1.0);
    }

    public MechanismSpec(
        String robot,
        String mechanism,
        int motorId,
        String canBus,
        double statorLimitAmps,
        double supplyLimitAmps,
        double slipCurrentAmps,
        String gravityType,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        double minPosRot,
        double maxPosRot) {
      this(
          robot,
          mechanism,
          motorId,
          canBus,
          statorLimitAmps,
          supplyLimitAmps,
          slipCurrentAmps,
          gravityType,
          kP,
          kI,
          kD,
          kS,
          kV,
          kA,
          kG,
          minPosRot,
          maxPosRot,
          -1,
          false,
          false,
          1.0);
    }

    /** Returns a copy of this {@link MechanismSpec} with an updated {@code canBus}. */
    public MechanismSpec withCanBus(String resolvedCanBus) {
      if (canBus.equals(resolvedCanBus)) {
        return this;
      }
      return new MechanismSpec(
          robot,
          mechanism,
          motorId,
          resolvedCanBus,
          statorLimitAmps,
          supplyLimitAmps,
          slipCurrentAmps,
          gravityType,
          kP,
          kI,
          kD,
          kS,
          kV,
          kA,
          kG,
          minPosRot,
          maxPosRot,
          followerMotorId,
          invertFollower,
          motorInverted,
          gearRatio);
    }
  }

  public record CameraSpec(
      String robot,
      @RowKey String cameraName,
      int portIndex,
      double xMeters,
      double yMeters,
      double zMeters,
      double rollDeg,
      double pitchDeg,
      double yawDeg,
      @Tunable double stdDevScalar) {}

  public record BatterySpec(
      @RowKey String batteryName,
      String qrCode,
      int purchaseYear,
      double nominalRintMilliOhms,
      double maxAllowedRintMilliOhms) {}

  public record ReleaseEntry(
      @RowKey String version,
      String configSha256,
      String timestamp,
      String author,
      String notes) {}

  record OverrideKey(String profile, String key) {}

  record TableCellOverrideKey(String tableName, String profile, String rowKey, String column) {}

  private record BoundTuningTopic(
      String topic,
      boolean isTableCell,
      String tableName,
      String profile,
      String rowKey,
      String columnOrKey,
      String publishedBaseline) {}

  private final Path configDir;
  private final Map<String, String> rawCsvFiles = new LinkedHashMap<>();
  private final Map<String, List<String>> parameterHeaders = new LinkedHashMap<>();
  private final Map<String, String> parameterKeySourceFile = new LinkedHashMap<>();
  private final Map<String, String> parameterCanonicalKeys = new LinkedHashMap<>();
  private final Map<String, Map<String, String>> parameterTable = new LinkedHashMap<>();
  private final Map<String, List<Map<String, String>>> relationalTables = new LinkedHashMap<>();
  private final Map<OverrideKey, String> runtimeOverrides = new LinkedHashMap<>();
  private final Map<TableCellOverrideKey, String> tableCellOverrides = new LinkedHashMap<>();
  private final Map<String, BoundTuningTopic> boundTuningTopics = new LinkedHashMap<>();
  private final Set<String> registeredTunableParamKeys = new LinkedHashSet<>();
  private final Map<String, Class<? extends Record>> registeredTableSchemas =
      new LinkedHashMap<>();
  private final Map<String, MechanismSpec> mechanismSpecs = new LinkedHashMap<>();
  private final List<MechanismSpec> allMechanismRows = new ArrayList<>();
  private final List<CameraSpec> cameraSpecs = new ArrayList<>();
  private final Map<String, BatterySpec> batterySpecs = new LinkedHashMap<>();
  private final List<BatterySpec> allBatteryRows = new ArrayList<>();
  private final List<ReleaseEntry> releases = new ArrayList<>();
  private final List<String> csvParseErrors = new ArrayList<>();
  private final List<String> parseErrors = new ArrayList<>();
  private final List<Consumer<FluxFig>> reloadListeners = new ArrayList<>();

  private String activeProfile = "DEFAULT";
  private Mode mode = Mode.PRODUCTION;
  private boolean productionLocked = false;
  private boolean fmsLocked = false;
  private TuningBackend tuningBackend = null;
  private Path tmpTunedSnapshotPath = null;
  private String configSha256 = "00000000";
  private long lastObservedDiskFingerprint = 0L;

  public FluxFig(Path configDir) {
    this.configDir = configDir;
    reload();
  }

  private FluxFig(Map<String, String> inMemoryCsvFiles, String activeProfile) {
    this.configDir = null;
    this.activeProfile = FluxFigCsvCodec.normalizeProfileHeader(activeProfile);
    loadFromCsvMapInternal(inMemoryCsvFiles);
  }

  /** Static factory for loading {@link FluxFig} in {@link Mode#PRODUCTION} from a directory. */
  public static FluxFig load(Path configDir) {
    return new FluxFig(configDir);
  }

  /**
   * Static factory for loading {@link FluxFig} in {@link Mode#PRODUCTION} and setting {@code
   * activeProfile}.
   */
  public static FluxFig load(Path configDir, String activeProfile) {
    FluxFig fig = new FluxFig(configDir);
    fig.setActiveProfile(activeProfile);
    return fig;
  }

  /** Explicit factory for loading a locked {@link Mode#PRODUCTION} {@link FluxFig} instance. */
  public static FluxFig loadProduction(Path configDir, String activeProfile) {
    FluxFig fig = new FluxFig(configDir);
    fig.setActiveProfile(activeProfile);
    fig.setMode(Mode.PRODUCTION);
    fig.productionLocked = true;
    return fig;
  }

  /**
   * Explicit factory for loading a {@link Mode#TUNING} {@link FluxFig} instance wired to a {@link
   * TuningBackend} (e.g., NetworkTables / Elastic on the robot or {@link InMemoryTuningBackend} in
   * tests).
   */
  public static FluxFig loadTuning(
      Path configDir, String activeProfile, TuningBackend tuningBackend) {
    FluxFig fig = new FluxFig(configDir);
    fig.setActiveProfile(activeProfile);
    fig.enableTuning(tuningBackend);
    return fig;
  }

  /**
   * Creates a {@link FluxFig} instance directly from an in-memory map of {@code tableName ->
   * csvContent} (with zero disk I/O required).
   */
  public static FluxFig fromCsvMap(Map<String, String> csvFiles) {
    return new FluxFig(csvFiles, "DEFAULT");
  }

  /**
   * Creates a {@link FluxFig} instance directly from an in-memory map of {@code tableName ->
   * csvContent} and sets {@code activeProfile}.
   */
  public static FluxFig fromCsvMap(Map<String, String> csvFiles, String activeProfile) {
    return new FluxFig(csvFiles, activeProfile);
  }

  /**
   * Reconstructs an in-memory {@link FluxFig} instance (including {@code Mode}, {@code
   * ActiveProfile}, all CSV tables, {@code RuntimeOverrides}, and {@code TableCellOverrides})
   * directly from a {@code /FluxFig/EffectiveSnapshot} or {@code /tmp/fluxfig-tuned-snapshot.txt}
   * string recorded by {@link #getEffectiveSnapshot()}.
   */
  public static FluxFig fromEffectiveSnapshot(String snapshotText) {
    return FluxFigCsvCodec.parseEffectiveSnapshot(snapshotText);
  }

  /**
   * Loads a {@link FluxFig} instance directly from a Google Sheet (spreadsheet ID or full URL) into
   * memory without writing to disk.
   */
  public static FluxFig loadFromGoogleSheet(String sheetIdOrUrl, List<String> tabNames) {
    return loadFromGoogleSheet(
        sheetIdOrUrl, tabNames, "DEFAULT", FluxFigSheetSync::fetchTabViaHttp);
  }

  /**
   * Loads a {@link FluxFig} instance directly from a Google Sheet (spreadsheet ID or full URL) with
   * a custom {@link SheetTabFetcher} and active profile.
   */
  public static FluxFig loadFromGoogleSheet(
      String sheetIdOrUrl,
      List<String> tabNames,
      String activeProfile,
      SheetTabFetcher tabFetcher) {
    try {
      Map<String, String> downloaded =
          FluxFigSheetSync.downloadSheetTabs(sheetIdOrUrl, tabNames, tabFetcher);
      return new FluxFig(downloaded, activeProfile);
    } catch (IOException e) {
      throw new IllegalStateException(
          "Failed to load FluxFig from Google Sheet " + sheetIdOrUrl, e);
    }
  }

  /**
   * Pulls CSV tabs from a Google Sheet configured in {@code configDir/fluxfig-sheet.properties}
   * (or the {@code FLUXFIG_SHEET_ID} environment variable), validates them with {@link
   * #validateStrict()}, atomically writes {@code *.csv} files into {@code configDir}, and returns
   * the loaded {@link FluxFig}.
   */
  public static FluxFig pullFromGoogleSheet(Path configDir) throws IOException {
    return FluxFigSheetSync.pullFromGoogleSheet(configDir);
  }

  /**
   * Downloads the specified tabs from a Google Sheet (or discovers existing tab names in {@code
   * configDir} if {@code tabNames} is empty), validates the candidate configuration via {@link
   * #validateStrict()}, writes normalized UTF-8 {@code *.csv} files to {@code configDir} only if
   * validation succeeds, and returns the resulting {@link FluxFig}.
   */
  public static FluxFig pullFromGoogleSheet(
      Path configDir, String sheetIdOrUrl, List<String> tabNames) throws IOException {
    return FluxFigSheetSync.pullFromGoogleSheet(
        configDir, sheetIdOrUrl, tabNames, FluxFigSheetSync::fetchTabViaHttp);
  }

  /**
   * Downloads tabs using the provided {@link SheetTabFetcher}, runs {@link #validateStrict()}
   * before touching {@code configDir}, and writes the validated {@code *.csv} files to {@code
   * configDir}.
   */
  public static FluxFig pullFromGoogleSheet(
      Path configDir,
      String sheetIdOrUrl,
      List<String> tabNames,
      SheetTabFetcher tabFetcher)
      throws IOException {
    return FluxFigSheetSync.pullFromGoogleSheet(configDir, sheetIdOrUrl, tabNames, tabFetcher);
  }

  /**
   * Attempts to sync {@code configDir} from Google Sheets when network is available, and falls back
   * seamlessly to the local offline {@code configDir/*.csv} files if offline or unreachable (e.g.,
   * in the competition pit).
   */
  public static FluxFig loadWithSheetSync(
      Path configDir, String sheetIdOrUrl, List<String> tabNames) {
    try {
      return pullFromGoogleSheet(configDir, sheetIdOrUrl, tabNames);
    } catch (IOException | RuntimeException e) {
      return load(configDir);
    }
  }

  /**
   * Extracts the canonical Google Spreadsheet ID from a full {@code docs.google.com/spreadsheets/d/}
   * URL, or returns the trimmed ID directly.
   */
  public static String extractSpreadsheetId(String sheetIdOrUrl) {
    return FluxFigSheetSync.extractSpreadsheetId(sheetIdOrUrl);
  }

  /**
   * Builds the CSV export URI for a given Google Sheet ID/URL (or custom HTTP base URL for local
   * testing) and tab name.
   */
  public static URI buildSheetCsvExportUri(String sheetIdOrUrl, String tabName) {
    return FluxFigSheetSync.buildSheetCsvExportUri(sheetIdOrUrl, tabName);
  }

  /**
   * Formats the contents of {@code deploy/configs/active-profile.txt} for {@code FluxTray} SFTP
   * profile &amp; mode switching ({@code PRODUCTION} vs. {@code TUNING}).
   */
  public static String formatActiveProfileDirective(
      String profile, Mode mode, String configSha256) {
    return FluxFigCsvCodec.formatActiveProfileDirective(profile, mode, configSha256);
  }

  /**
   * Parses {@code deploy/configs/active-profile.txt} written by {@code FluxTray} (supporting both
   * the legacy 2-line {@code <profile>\nConfigSHA256=...} format and the {@code mode=TUNING|PRODUCTION}
   * directive).
   */
  public static ActiveProfileDirective parseActiveProfileDirective(String rawText) {
    return FluxFigCsvCodec.parseActiveProfileDirective(rawText);
  }

  /**
   * Reloads all {@code *.csv} files (and {@code active-profile.txt} if present) from {@code
   * configDir} in deterministic alphabetical order, auto-classifying Key-Value Matrix tabs ({@code
   * key} in column 0) vs. Relational Table tabs, clearing stale runtime overrides, and recomputing
   * the 8-character {@code ConfigSHA256}.
   */
  public synchronized void reload() {
    if (configDir == null) {
      throw new IllegalStateException("Cannot reload from disk on an in-memory FluxFig instance");
    }
    try {
      if (!Files.isDirectory(configDir)) {
        throw new IOException("Config directory does not exist: " + configDir);
      }
      List<Path> csvFiles;
      try (Stream<Path> stream = Files.list(configDir)) {
        csvFiles =
            stream
                .filter(Files::isRegularFile)
                .filter(
                    p ->
                        p.getFileName()
                            .toString()
                            .toLowerCase(Locale.ROOT)
                            .endsWith(".csv"))
                .sorted(Comparator.comparing(p -> p.getFileName().toString()))
                .toList();
      }

      Map<String, String> loadedFiles = new LinkedHashMap<>();
      for (Path csvPath : csvFiles) {
        String fileName = csvPath.getFileName().toString();
        String content = Files.readString(csvPath, StandardCharsets.UTF_8);
        loadedFiles.put(fileName, content);
      }

      runtimeOverrides.clear();
      tableCellOverrides.clear();
      boundTuningTopics.clear();

      loadFromCsvMapInternal(loadedFiles);

      Path activeProfilePath = configDir.resolve(ACTIVE_PROFILE_FILE);
      if (Files.isRegularFile(activeProfilePath)) {
        ActiveProfileDirective dir =
            parseActiveProfileDirective(
                Files.readString(activeProfilePath, StandardCharsets.UTF_8));
        if (!dir.profile().isEmpty()) {
          this.activeProfile = dir.profile();
        }
        if (!this.fmsLocked) {
          setMode(dir.mode());
        }
      }

      this.lastObservedDiskFingerprint = computeDiskFingerprint();
      if (this.mode == Mode.TUNING && this.tuningBackend != null) {
        publishRegisteredTunableParameters();
      }
      for (Consumer<FluxFig> listener : reloadListeners) {
        listener.accept(this);
      }
    } catch (IOException e) {
      throw new IllegalStateException("Failed to load FluxFig CSVs from " + configDir, e);
    }
  }

  /**
   * Checks whether any {@code *.csv} file or {@code active-profile.txt} in {@code configDir} has
   * been updated on disk (e.g., via {@code FluxTray} SFTP push while Disabled), and calls {@link
   * #reload()} only if a change occurred.
   *
   * @return {@code true} if files changed and {@link #reload()} was executed
   */
  public synchronized boolean reloadIfChanged() {
    if (configDir == null || !Files.isDirectory(configDir)) {
      return false;
    }
    long currentFingerprint = computeDiskFingerprint();
    if (currentFingerprint != lastObservedDiskFingerprint) {
      reload();
      return true;
    }
    return false;
  }

  /** Registers a callback invoked whenever {@link #reload()} or {@link #reloadIfChanged()} runs. */
  public synchronized void onReload(Consumer<FluxFig> listener) {
    reloadListeners.add(Objects.requireNonNull(listener));
  }

  private long computeDiskFingerprint() {
    if (configDir == null || !Files.isDirectory(configDir)) {
      return 0L;
    }
    long hash = 1125899906842597L;
    try (Stream<Path> s = Files.list(configDir)) {
      for (Path p :
          s.filter(Files::isRegularFile)
              .filter(
                  f -> {
                    String n = f.getFileName().toString().toLowerCase(Locale.ROOT);
                    return n.endsWith(".csv") || n.equals(ACTIVE_PROFILE_FILE);
                  })
              .sorted(Comparator.comparing(f -> f.getFileName().toString()))
              .toList()) {
        hash = 31L * hash + p.getFileName().toString().hashCode();
        hash = 31L * hash + Files.getLastModifiedTime(p).toMillis();
        hash = 31L * hash + Files.size(p);
      }
    } catch (IOException ignored) {
    }
    return hash;
  }

  private synchronized void loadFromCsvMapInternal(Map<String, String> filesMap) {
    rawCsvFiles.clear();
    parameterHeaders.clear();
    parameterKeySourceFile.clear();
    parameterCanonicalKeys.clear();
    parameterTable.clear();
    relationalTables.clear();
    mechanismSpecs.clear();
    allMechanismRows.clear();
    cameraSpecs.clear();
    batterySpecs.clear();
    allBatteryRows.clear();
    releases.clear();
    csvParseErrors.clear();
    parseErrors.clear();

    List<String> sortedFileNames = new ArrayList<>(filesMap.keySet());
    sortedFileNames.sort(Comparator.naturalOrder());

    for (String rawName : sortedFileNames) {
      String tableName = FluxFigCsvCodec.stripCsvExtension(rawName);
      String fileName =
          rawName.toLowerCase(Locale.ROOT).endsWith(".csv") ? rawName : rawName + ".csv";
      String normalizedContent = FluxFigCsvCodec.normalizeLineEndings(filesMap.get(rawName));
      rawCsvFiles.put(fileName, normalizedContent);
      FluxFigCsvCodec.parseAndClassifyCsv(
          tableName,
          normalizedContent,
          parameterHeaders,
          parameterCanonicalKeys,
          parameterKeySourceFile,
          parameterTable,
          relationalTables,
          csvParseErrors);
    }

    hydrateBuiltInSpecs();
    this.configSha256 = FluxFigCsvCodec.computeConfigSha256(rawCsvFiles);
  }

  String resolveCanonicalParameterKey(String rawKey) {
    if (rawKey == null) {
      return "";
    }
    String trimmed = rawKey.trim();
    if (parameterTable.containsKey(trimmed)) {
      return trimmed;
    }
    String norm = FluxFigCsvCodec.normalizeIdentifier(trimmed);
    return parameterCanonicalKeys.getOrDefault(norm, trimmed);
  }

  private void hydrateBuiltInSpecs() {
    mechanismSpecs.clear();
    allMechanismRows.clear();
    cameraSpecs.clear();
    batterySpecs.clear();
    allBatteryRows.clear();
    releases.clear();

    for (MechanismSpec rawSpec :
        FluxFigRecordBinder.bindRowsToRecord(
            "mechanisms.csv",
            getTableWithCellOverrides("mechanisms"),
            MechanismSpec.class,
            true,
            parseErrors)) {
      MechanismSpec spec =
          new MechanismSpec(
              FluxFigCsvCodec.normalizeProfileHeader(rawSpec.robot()),
              rawSpec.mechanism(),
              rawSpec.motorId(),
              rawSpec.canBus(),
              rawSpec.statorLimitAmps(),
              rawSpec.supplyLimitAmps(),
              rawSpec.slipCurrentAmps(),
              rawSpec.gravityType(),
              rawSpec.kP(),
              rawSpec.kI(),
              rawSpec.kD(),
              rawSpec.kS(),
              rawSpec.kV(),
              rawSpec.kA(),
              rawSpec.kG(),
              rawSpec.minPosRot(),
              rawSpec.maxPosRot(),
              rawSpec.followerMotorId(),
              rawSpec.invertFollower(),
              rawSpec.motorInverted(),
              rawSpec.gearRatio());
      allMechanismRows.add(spec);
      mechanismSpecs.put(spec.robot() + ":" + spec.mechanism(), spec);
    }

    for (CameraSpec cam :
        FluxFigRecordBinder.bindRowsToRecord(
            "cameras.csv",
            getTableWithCellOverrides("cameras"),
            CameraSpec.class,
            true,
            parseErrors)) {
      cameraSpecs.add(
          new CameraSpec(
              FluxFigCsvCodec.normalizeProfileHeader(cam.robot()),
              cam.cameraName(),
              cam.portIndex(),
              cam.xMeters(),
              cam.yMeters(),
              cam.zMeters(),
              cam.rollDeg(),
              cam.pitchDeg(),
              cam.yawDeg(),
              cam.stdDevScalar()));
    }

    for (BatterySpec bat :
        FluxFigRecordBinder.bindRowsToRecord(
            "batteries.csv",
            getTableWithCellOverrides("batteries"),
            BatterySpec.class,
            true,
            parseErrors)) {
      allBatteryRows.add(bat);
      batterySpecs.put(bat.batteryName(), bat);
    }

    for (ReleaseEntry rel :
        FluxFigRecordBinder.bindRowsToRecord(
            "releases.csv",
            getTableWithCellOverrides("releases"),
            ReleaseEntry.class,
            true,
            parseErrors)) {
      releases.add(rel);
    }
  }

  /**
   * Validates hardware safety constraints across all loaded CSV tables, parameter overrides, and
   * table cell overrides.
   *
   * @return list of validation errors (empty if valid)
   */
  public synchronized List<String> validateStrict() {
    List<String> combinedParseErrors = new ArrayList<>(csvParseErrors.size() + parseErrors.size());
    combinedParseErrors.addAll(csvParseErrors);
    combinedParseErrors.addAll(parseErrors);
    return FluxFigValidator.validateStrict(
        this,
        rawCsvFiles,
        combinedParseErrors,
        parameterHeaders,
        parameterTable,
        runtimeOverrides,
        relationalTables,
        registeredTableSchemas,
        allMechanismRows,
        cameraSpecs,
        allBatteryRows);
  }

  public synchronized Mode getMode() {
    return mode;
  }

  /**
   * Switches the operating mode between {@link Mode#PRODUCTION} and {@link Mode#TUNING}. Switching
   * to {@link Mode#PRODUCTION} clears temporary tuning bindings so values are locked to CSVs. When
   * FMS lockout ({@link #lockProductionOnFms(boolean)}) is active, switching to {@link Mode#TUNING}
   * is ignored.
   */
  public synchronized void setMode(Mode newMode) {
    Objects.requireNonNull(newMode);
    if (this.fmsLocked && newMode == Mode.TUNING) {
      return;
    }
    this.mode = newMode;
    if (newMode == Mode.PRODUCTION) {
      boundTuningTopics.clear();
      this.productionLocked = true;
    } else {
      this.productionLocked = false;
      if (tuningBackend != null) {
        publishRegisteredTunableParameters();
      }
    }
  }

  /**
   * Enables {@link Mode#TUNING} with the specified {@link TuningBackend} (e.g., NetworkTables /
   * Elastic on the robot or {@link InMemoryTuningBackend} in tests) and publishes registered
   * tunable parameters. Ignored while FMS lockout is active.
   */
  public synchronized void enableTuning(TuningBackend backend) {
    this.tuningBackend = Objects.requireNonNull(backend);
    if (this.fmsLocked) {
      return;
    }
    this.mode = Mode.TUNING;
    this.productionLocked = false;
    publishRegisteredTunableParameters();
  }

  /**
   * Disables {@link Mode#TUNING}, clears all temporary in-memory overrides, and locks this
   * instance to {@link Mode#PRODUCTION}.
   */
  public synchronized void disableTuningAndDiscardOverrides() {
    this.mode = Mode.PRODUCTION;
    this.productionLocked = true;
    this.boundTuningTopics.clear();
    clearOverrides();
  }

  /**
   * Enforces competition FMS safety lockout: when {@code isFmsAttached == true}, immediately forces
   * {@link Mode#PRODUCTION} and clears any uncommitted temporary overrides so the robot runs pure,
   * locked Git CSV constants on the competition field.
   */
  public synchronized void lockProductionOnFms(boolean isFmsAttached) {
    this.fmsLocked = isFmsAttached;
    if (isFmsAttached) {
      this.productionLocked = true;
      if (this.mode != Mode.PRODUCTION || isModified()) {
        disableTuningAndDiscardOverrides();
      }
    }
  }

  /**
   * Configures the temporary RAM-disk file path (e.g., {@code /tmp/fluxfig-tuned-snapshot.txt})
   * where {@link #pollTuning()} automatically writes the updated snapshot whenever an override
   * changes.
   */
  public synchronized void setTmpTunedSnapshotPath(Path tmpSnapshotPath) {
    this.tmpTunedSnapshotPath = tmpSnapshotPath;
  }

  /**
   * Writes the current {@link #getEffectiveSnapshot()} to {@code targetFile} (such as {@code
   * /tmp/fluxfig-tuned-snapshot.txt} on the robot when transitioning to Disabled) so {@code
   * FluxTray} can pull it over SFTP.
   */
  public synchronized void writeTunedSnapshot(Path targetFile) throws IOException {
    if (targetFile.getParent() != null) {
      Files.createDirectories(targetFile.getParent());
    }
    Files.writeString(targetFile, getEffectiveSnapshot(), StandardCharsets.UTF_8);
  }

  /**
   * Explicitly registers specific {@code parameters.csv} keys (or all non-structural numeric keys
   * if {@code keys} is empty) to be published to the dashboard in {@link Mode#TUNING}.
   */
  public synchronized void registerTunableParameters(String... keys) {
    if (keys == null || keys.length == 0) {
      for (String k : parameterTable.keySet()) {
        if (!STRUCTURAL_PARAMETER_KEYS.contains(FluxFigCsvCodec.normalizeIdentifier(k))) {
          registeredTunableParamKeys.add(k);
        }
      }
    } else {
      for (String k : keys) {
        if (k != null && !k.isBlank()) {
          registeredTunableParamKeys.add(resolveCanonicalParameterKey(k));
        }
      }
    }
    if (mode == Mode.TUNING && tuningBackend != null) {
      publishRegisteredTunableParameters();
    }
  }

  private void publishRegisteredTunableParameters() {
    if (mode != Mode.TUNING || tuningBackend == null) {
      return;
    }
    Set<String> keysToPublish =
        registeredTunableParamKeys.isEmpty()
            ? defaultTunableParameterKeys()
            : registeredTunableParamKeys;
    for (String rawKey : keysToPublish) {
      String canonicalKey = resolveCanonicalParameterKey(rawKey);
      Optional<String> valOpt = resolveEffectiveParameter(activeProfile, canonicalKey);
      if (valOpt.isPresent()) {
        String topic = TUNING_TOPIC_PREFIX + "parameters/" + canonicalKey;
        String val = valOpt.get();
        tuningBackend.publish(topic, val);
        boundTuningTopics.put(
            topic,
            new BoundTuningTopic(
                topic, false, "parameters", activeProfile, "", canonicalKey, val));
      }
    }
  }

  private Set<String> defaultTunableParameterKeys() {
    Set<String> out = new LinkedHashSet<>();
    for (String k : parameterTable.keySet()) {
      if (!STRUCTURAL_PARAMETER_KEYS.contains(FluxFigCsvCodec.normalizeIdentifier(k))) {
        Optional<String> val = resolveEffectiveParameter(activeProfile, k);
        if (val.isPresent() && isNumericLiteral(val.get())) {
          out.add(k);
        }
      }
    }
    return out;
  }

  private static boolean isNumericLiteral(String s) {
    try {
      Double.parseDouble(s.trim());
      return true;
    } catch (NumberFormatException e) {
      return false;
    }
  }

  /**
   * Polls all bound {@link Tunable} dashboard topics when in {@link Mode#TUNING}, applies any
   * user-edited values to in-memory parameter or relational table overrides, and updates {@code
   * tmpTunedSnapshotPath} if configured. In {@link Mode#PRODUCTION}, this method is a fast no-op.
   *
   * @return number of parameters/cells that changed on this poll cycle
   */
  public synchronized int pollTuning() {
    if (mode != Mode.TUNING || tuningBackend == null || boundTuningTopics.isEmpty()) {
      return 0;
    }
    int changedCount = 0;
    for (Map.Entry<String, BoundTuningTopic> entry :
        new ArrayList<>(boundTuningTopics.entrySet())) {
      BoundTuningTopic bound = entry.getValue();
      Optional<String> latestOpt = tuningBackend.read(bound.topic());
      if (latestOpt.isEmpty()) {
        continue;
      }
      String latest = latestOpt.get().trim();
      if (!latest.isEmpty() && !latest.equals(bound.publishedBaseline())) {
        if (bound.isTableCell()) {
          setTableCellOverride(
              bound.tableName(), bound.profile(), bound.rowKey(), bound.columnOrKey(), latest);
        } else {
          setOverride(bound.profile(), bound.columnOrKey(), latest);
        }
        boundTuningTopics.put(
            bound.topic(),
            new BoundTuningTopic(
                bound.topic(),
                bound.isTableCell(),
                bound.tableName(),
                bound.profile(),
                bound.rowKey(),
                bound.columnOrKey(),
                latest));
        changedCount++;
      }
    }
    if (changedCount > 0 && tmpTunedSnapshotPath != null) {
      try {
        writeTunedSnapshot(tmpTunedSnapshotPath);
      } catch (IOException ignored) {
      }
    }
    return changedCount;
  }

  /**
   * Sets an in-memory cell override on a relational table row (such as {@code range_table}, {@code
   * elevator_presets}, or {@code mechanisms}) for the currently active profile.
   */
  public synchronized void setTableCellOverride(
      String tableName, String rowKey, String column, Object value) {
    setTableCellOverride(tableName, activeProfile, rowKey, column, value);
  }

  /**
   * Sets an in-memory cell override on a relational table row for a specific {@code profile}.
   * Immediately re-hydrates built-in specs if the table is {@code mechanisms}, {@code cameras}, or
   * {@code batteries}.
   */
  public synchronized void setTableCellOverride(
      String tableName, String profile, String rowKey, String column, Object value) {
    if (productionLocked && mode == Mode.PRODUCTION) {
      throw new IllegalStateException(
          "Cannot setTableCellOverride('"
              + tableName
              + "', '"
              + rowKey
              + "', '"
              + column
              + "') while FluxFig is locked in Mode.PRODUCTION");
    }
    setTableCellOverrideUnchecked(tableName, profile, rowKey, column, value);
  }

  void setTableCellOverrideUnchecked(
      String tableName, String profile, String rowKey, String column, Object value) {
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    String normProf = FluxFigCsvCodec.normalizeProfileHeader(profile);
    tableCellOverrides.put(
        new TableCellOverrideKey(cleanTable, normProf, rowKey.trim(), column.trim()),
        String.valueOf(value).trim());
    if ("mechanisms".equals(cleanTable)
        || "cameras".equals(cleanTable)
        || "batteries".equals(cleanTable)) {
      parseErrors.clear();
      hydrateBuiltInSpecs();
    }
  }

  /**
   * Atomically updates {@code cameras.csv} for {@code robot} (preserving rows for all other robots
   * as well as any leading {@code #} comments and extra columns), validates the resulting
   * configuration via {@link #validateStrict()}, writes {@code cameras.csv} to {@code configDir} if
   * backed by disk, and reloads in-memory state.
   */
  public synchronized void saveCameraSpecs(String robot, List<CameraSpec> updatedSpecs)
      throws IOException {
    String serialized =
        FluxFigCsvCodec.serializeCameraSpecsCsv(
            rawCsvFiles.get("cameras.csv"), getTable("cameras"), robot, updatedSpecs);
    applyAndPersistTableUpdate("cameras.csv", serialized);
  }

  /**
   * Atomically updates {@code mechanisms.csv} for {@code robot} (preserving rows for all other
   * robots as well as any leading {@code #} comments and extra columns like {@code notes}),
   * validates the resulting configuration via {@link #validateStrict()}, writes {@code
   * mechanisms.csv} to {@code configDir} if backed by disk, and reloads in-memory state.
   */
  public synchronized void saveMechanismSpecs(String robot, List<MechanismSpec> updatedSpecs)
      throws IOException {
    String serialized =
        FluxFigCsvCodec.serializeMechanismSpecsCsv(
            rawCsvFiles.get("mechanisms.csv"), getTable("mechanisms"), robot, updatedSpecs);
    applyAndPersistTableUpdate("mechanisms.csv", serialized);
  }

  /**
   * Atomically updates {@code batteries.csv} (e.g., when {@code FluxProcessor --calibrate-sim}
   * writes measured internal resistance {@code nominalRintMilliOhms} back to the laptop CSV),
   * preserving any leading {@code #} comments and extra columns, validating via {@link
   * #validateStrict()}, and reloading state.
   */
  public synchronized void saveBatterySpecs(List<BatterySpec> updatedBatteries) throws IOException {
    String serialized =
        FluxFigCsvCodec.serializeBatterySpecsCsv(
            rawCsvFiles.get("batteries.csv"), getTable("batteries"), updatedBatteries);
    applyAndPersistTableUpdate("batteries.csv", serialized);
  }

  /**
   * Commits all active {@code runtimeOverrides} (into {@code parameters.csv}) and {@code
   * tableCellOverrides} (into their respective relational {@code *.csv} files), preserving {@code
   * #} comments and extra columns, validates via {@link #validateStrict()}, atomically writes
   * updated files to {@code configDir} if backed by disk, clears overrides, and recomputes {@code
   * ConfigSHA256}.
   */
  public synchronized void persistOverridesToCsv() throws IOException {
    if (runtimeOverrides.isEmpty() && tableCellOverrides.isEmpty()) {
      return;
    }
    Map<String, String> candidateFiles =
        FluxFigCsvCodec.buildCandidateFilesWithOverridesApplied(
            this,
            rawCsvFiles,
            runtimeOverrides,
            tableCellOverrides,
            parameterHeaders,
            parameterTable,
            parameterKeySourceFile);
    validateAndPersistCandidateFiles(candidateFiles);
    runtimeOverrides.clear();
    tableCellOverrides.clear();
  }

  /**
   * Records a new immutable release entry in {@code releases.csv} for the current validated {@code
   * ConfigSHA256}. If {@code version} is blank or {@code "auto"}, automatically increments the
   * highest existing {@code vN} tag in {@code releases.csv} (e.g. {@code v3 -> v4}, or {@code v1}
   * if none exist).
   */
  public synchronized ReleaseEntry recordRelease(String version, String author, String notes)
      throws IOException {
    List<String> errors = validateStrict();
    if (!errors.isEmpty()) {
      throw new IllegalStateException(
          "Cannot record release on invalid configuration (" + errors.size() + " errors): " + errors);
    }
    String effectiveVersionTag =
        (version == null || version.isBlank() || "auto".equalsIgnoreCase(version.trim()))
            ? FluxFigCsvCodec.computeNextReleaseVersionTag(releases)
            : version.trim();
    String effectiveAuthor = (author == null || author.isBlank()) ? "flux-team" : author.trim();
    String effectiveNotes = (notes == null) ? "" : notes.trim();
    String nowIso = Instant.now().toString();

    ReleaseEntry newEntry =
        new ReleaseEntry(
            effectiveVersionTag, configSha256, nowIso, effectiveAuthor, effectiveNotes);

    String serialized =
        FluxFigCsvCodec.serializeReleaseEntryCsv(
            rawCsvFiles.get("releases.csv"), getTable("releases"), newEntry);
    applyAndPersistTableUpdate("releases.csv", serialized);
    return newEntry;
  }

  /**
   * End-of-session version harvester for {@code FluxTray} and CLI ({@code fluxfig.jar promote}):
   * takes a {@code /FluxFig/EffectiveSnapshot} or {@code /tmp/fluxfig-tuned-snapshot.txt} string,
   * applies all parameter and table-cell overrides to {@code targetConfigDir/*.csv} (preserving
   * comments and custom columns), validates via {@link #validateStrict()}, computes the new {@code
   * ConfigSHA256}, and appends the next {@code vN} version row to {@code releases.csv} with an
   * automatic diff summary.
   */
  public static ReleaseEntry promoteSnapshotToDirectory(
      String snapshotText, Path targetConfigDir, String author, String customNotes)
      throws IOException {
    FluxFig snapshotFig = fromEffectiveSnapshot(snapshotText);
    Map<String, String> diffMap = snapshotFig.getModifiedDiff();
    String autoDiffSummary = FluxFigCsvCodec.formatDiffSummary(diffMap);

    FluxFig targetFig =
        (targetConfigDir != null && Files.isDirectory(targetConfigDir))
            ? new FluxFig(targetConfigDir)
            : snapshotFig;
    if (targetFig != snapshotFig) {
      Map<String, String> baseFiles = new LinkedHashMap<>(targetFig.rawCsvFiles);
      for (Map.Entry<String, String> e : snapshotFig.rawCsvFiles.entrySet()) {
        baseFiles.putIfAbsent(e.getKey(), e.getValue());
      }
      targetFig.loadFromCsvMapInternal(baseFiles);
      targetFig.setActiveProfile(snapshotFig.getActiveProfile());
      for (Map.Entry<OverrideKey, String> ov : snapshotFig.runtimeOverrides.entrySet()) {
        targetFig.setOverrideUnchecked(ov.getKey().profile(), ov.getKey().key(), ov.getValue());
      }
      for (Map.Entry<TableCellOverrideKey, String> tov :
          snapshotFig.tableCellOverrides.entrySet()) {
        TableCellOverrideKey k = tov.getKey();
        targetFig.setTableCellOverrideUnchecked(
            k.tableName(), k.profile(), k.rowKey(), k.column(), tov.getValue());
      }
    }

    targetFig.persistOverridesToCsv();
    targetFig.setMode(Mode.PRODUCTION);

    String combinedNotes;
    if (customNotes == null || customNotes.isBlank()) {
      combinedNotes =
          autoDiffSummary.isEmpty() ? "Promoted snapshot" : "Tuned: " + autoDiffSummary;
    } else if (autoDiffSummary.isEmpty()) {
      combinedNotes = customNotes.trim();
    } else {
      combinedNotes = customNotes.trim() + " [" + autoDiffSummary + "]";
    }

    return targetFig.recordRelease("auto", author, combinedNotes);
  }

  private void applyAndPersistTableUpdate(String fileName, String newCsvContent)
      throws IOException {
    Map<String, String> candidateFiles = new LinkedHashMap<>(rawCsvFiles);
    candidateFiles.put(fileName, FluxFigCsvCodec.normalizeLineEndings(newCsvContent));
    validateAndPersistCandidateFiles(candidateFiles);
  }

  private void validateAndPersistCandidateFiles(Map<String, String> candidateFiles)
      throws IOException {
    FluxFig candidate = new FluxFig(candidateFiles, activeProfile);
    candidate.registeredTableSchemas.putAll(this.registeredTableSchemas);
    List<String> errors = candidate.validateStrict();
    if (!errors.isEmpty()) {
      throw new IllegalStateException(
          "Safety validation failed when updating CSV configuration ("
              + errors.size()
              + " errors): "
              + errors);
    }

    if (configDir != null) {
      Files.createDirectories(configDir);
      for (Map.Entry<String, String> entry : candidateFiles.entrySet()) {
        String existing = rawCsvFiles.get(entry.getKey());
        if (!entry.getValue().equals(existing)
            || !Files.isRegularFile(configDir.resolve(entry.getKey()))) {
          FluxFigCsvCodec.writeCsvFileAtomically(configDir, entry.getKey(), entry.getValue());
        }
      }
      loadFromCsvMapInternal(candidateFiles);
      this.lastObservedDiskFingerprint = computeDiskFingerprint();
    } else {
      loadFromCsvMapInternal(candidateFiles);
    }
  }

  public synchronized void setActiveProfile(String profile) {
    this.activeProfile = FluxFigCsvCodec.normalizeProfileHeader(profile);
    if (mode == Mode.TUNING && tuningBackend != null) {
      publishRegisteredTunableParameters();
    }
  }

  public synchronized String getActiveProfile() {
    return activeProfile;
  }

  public synchronized String getConfigSha256() {
    return configSha256;
  }

  /**
   * Sets an in-memory live-tuning or pit parameter override on the currently active profile,
   * marking {@link #isModified()} true and appending {@code -MODIFIED} to {@link
   * #getEffectiveVersion()}.
   */
  public synchronized void setOverride(String key, Object value) {
    setOverride(activeProfile, key, value);
  }

  /**
   * Sets an in-memory live-tuning or pit parameter override on a specific profile. Also supports
   * shorthand {@code "<table>:<rowKey>.<column>"} (or {@code "<MechanismName>.<field>"} for {@code
   * mechanisms.csv}) to override relational table cells directly.
   */
  public synchronized void setOverride(String profile, String key, Object value) {
    if (productionLocked && mode == Mode.PRODUCTION) {
      throw new IllegalStateException(
          "Cannot setOverride('"
              + profile
              + "', '"
              + key
              + "') while FluxFig is locked in Mode.PRODUCTION");
    }
    setOverrideUnchecked(profile, key, value);
  }

  void setOverrideUnchecked(String profile, String key, Object value) {
    String trimmedKey = key.trim();
    String normProf = FluxFigCsvCodec.normalizeProfileHeader(profile);

    int colonIdx = trimmedKey.indexOf(':');
    int dotIdx = trimmedKey.lastIndexOf('.');
    if (colonIdx > 0 && dotIdx > colonIdx) {
      String candidateTable =
          FluxFigCsvCodec.stripCsvExtension(trimmedKey.substring(0, colonIdx))
              .toLowerCase(Locale.ROOT);
      if (relationalTables.containsKey(candidateTable)) {
        String rowKey = trimmedKey.substring(colonIdx + 1, dotIdx).trim();
        String col = trimmedKey.substring(dotIdx + 1).trim();
        setTableCellOverrideUnchecked(candidateTable, normProf, rowKey, col, value);
        return;
      }
    }

    String canonicalKey = resolveCanonicalParameterKey(trimmedKey);
    if (dotIdx > 0 && !parameterTable.containsKey(canonicalKey)) {
      String prefix = trimmedKey.substring(0, dotIdx).trim();
      String field = trimmedKey.substring(dotIdx + 1).trim();
      if (hasMechanismWithName(normProf, prefix) && isMechanismSpecField(field)) {
        setTableCellOverrideUnchecked("mechanisms", normProf, prefix, field, value);
        return;
      }
    }

    runtimeOverrides.put(new OverrideKey(normProf, canonicalKey), String.valueOf(value));
  }

  private boolean hasMechanismWithName(String profile, String mechanismName) {
    for (String p : buildProfileHierarchy(profile)) {
      for (MechanismSpec m : allMechanismRows) {
        if (m.robot().equalsIgnoreCase(p) && m.mechanism().equalsIgnoreCase(mechanismName)) {
          return true;
        }
      }
    }
    return false;
  }

  private static boolean isMechanismSpecField(String fieldName) {
    String norm = FluxFigCsvCodec.normalizeIdentifier(fieldName);
    for (RecordComponent rc : MechanismSpec.class.getRecordComponents()) {
      if (FluxFigCsvCodec.normalizeIdentifier(rc.getName()).equals(norm)) {
        return true;
      }
    }
    return false;
  }

  /** Clears all in-memory parameter and table-cell overrides and restores pure CSV values. */
  public synchronized void clearOverrides() {
    boolean hadTableOverrides = !tableCellOverrides.isEmpty();
    runtimeOverrides.clear();
    tableCellOverrides.clear();
    if (hadTableOverrides) {
      parseErrors.clear();
      hydrateBuiltInSpecs();
    }
  }

  /**
   * Returns true if any active parameter or table-cell override differs from the base CSV
   * configuration.
   */
  public synchronized boolean isModified() {
    return !getModifiedDiff().isEmpty();
  }

  /**
   * Returns a map of {@code "<profile>.<key>" -> "<csvValue> -> <overrideValue>"} (and {@code
   * "<table>[<profile>:<rowKey>].<col>" -> "<csvValue> -> <overrideValue>"}) for every modified
   * runtime parameter or table cell.
   */
  public synchronized Map<String, String> getModifiedDiff() {
    Map<String, String> diffs = new LinkedHashMap<>();
    for (Map.Entry<OverrideKey, String> entry : runtimeOverrides.entrySet()) {
      String prof = entry.getKey().profile();
      String key = entry.getKey().key();
      String baseVal = resolveFromCsvOnly(prof, key).orElse("<unset>");
      if (!baseVal.equals(entry.getValue())) {
        diffs.put(prof + "." + key, baseVal + " -> " + entry.getValue());
      }
    }
    for (Map.Entry<TableCellOverrideKey, String> entry : tableCellOverrides.entrySet()) {
      TableCellOverrideKey tk = entry.getKey();
      String baseVal =
          resolveTableCellFromCsvOnly(tk.tableName(), tk.profile(), tk.rowKey(), tk.column())
              .orElse("<unset>");
      if (!baseVal.equals(entry.getValue())) {
        diffs.put(
            tk.tableName() + "[" + tk.profile() + ":" + tk.rowKey() + "]." + tk.column(),
            baseVal + " -> " + entry.getValue());
      }
    }
    return Collections.unmodifiableMap(diffs);
  }

  Optional<String> resolveTableCellFromCsvOnly(
      String tableName, String profile, String rowKey, String column) {
    return FluxFigRecordBinder.resolveTableCellFromCsvOnly(
        relationalTables, tableName, profile, rowKey, column);
  }

  /**
   * Returns the matching published {@link ReleaseEntry} from {@code releases.csv} if the current
   * {@code ConfigSHA256} matches a recorded release ledger row.
   */
  public synchronized Optional<ReleaseEntry> getMatchingRelease() {
    for (int i = releases.size() - 1; i >= 0; i--) {
      ReleaseEntry rel = releases.get(i);
      if (rel.configSha256().equalsIgnoreCase(configSha256)) {
        return Optional.of(rel);
      }
    }
    return Optional.empty();
  }

  /** Returns all rows from the optional {@code releases.csv} ledger. */
  public synchronized List<ReleaseEntry> getReleases() {
    return Collections.unmodifiableList(releases);
  }

  /**
   * Returns the human-readable version string (e.g., {@code "v2 (a1b2c3d4)"} or {@code
   * "a1b2c3d4-MODIFIED"}) for logging to AdvantageKit / {@code .wpilog} metadata.
   */
  public synchronized String getEffectiveVersion() {
    Optional<ReleaseEntry> rel = getMatchingRelease();
    String base = rel.map(r -> r.version() + " (" + configSha256 + ")").orElse(configSha256);
    return isModified() ? base + "-MODIFIED" : base;
  }

  /**
   * Builds a deterministic, self-contained snapshot of all loaded CSV tables plus any active
   * parameter and table-cell overrides suitable for stamping into {@code /FluxFig/EffectiveSnapshot}
   * in {@code .wpilog} files or {@code /tmp/fluxfig-tuned-snapshot.txt} for {@code FluxTray}.
   */
  public synchronized String getEffectiveSnapshot() {
    return FluxFigCsvCodec.formatEffectiveSnapshot(
        this,
        mode,
        getEffectiveVersion(),
        configSha256,
        activeProfile,
        runtimeOverrides,
        tableCellOverrides,
        rawCsvFiles);
  }

  /**
   * Returns true if {@code key} is defined for the currently active profile (or {@code DEFAULT}).
   */
  public synchronized boolean hasParameter(String key) {
    return resolveEffectiveParameter(activeProfile, key).isPresent();
  }

  /** Returns true if {@code key} is defined for {@code profile} (or {@code DEFAULT}). */
  public synchronized boolean hasParameter(String profile, String key) {
    return resolveEffectiveParameter(profile, key).isPresent();
  }

  /**
   * Resolves a parameter with hierarchical fallback inheritance: {@code <Profile>:<Overlay> ->
   * <Profile> -> DEFAULT}. Parameter key lookup is resilient to {@code camelCase}, {@code
   * snake_case}, and capitalization differences.
   */
  public synchronized String getString(String key) {
    return resolveEffectiveParameter(activeProfile, key)
        .orElseThrow(() -> new IllegalArgumentException("Missing FluxFig key: " + key));
  }

  Optional<String> resolveEffectiveParameter(String profile, String key) {
    String canonicalKey = resolveCanonicalParameterKey(key);
    Map<String, String> row = parameterTable.getOrDefault(canonicalKey, Collections.emptyMap());
    for (String candidateProfile : buildProfileHierarchy(profile)) {
      OverrideKey ok = new OverrideKey(candidateProfile, canonicalKey);
      if (runtimeOverrides.containsKey(ok)) {
        return Optional.of(runtimeOverrides.get(ok));
      }
      if (row.containsKey(candidateProfile)) {
        return Optional.of(row.get(candidateProfile));
      }
    }
    return Optional.empty();
  }

  Optional<String> resolveFromCsvOnly(String profile, String key) {
    String canonicalKey = resolveCanonicalParameterKey(key);
    Map<String, String> row = parameterTable.getOrDefault(canonicalKey, Collections.emptyMap());
    for (String candidateProfile : buildProfileHierarchy(profile)) {
      if (row.containsKey(candidateProfile)) {
        return Optional.of(row.get(candidateProfile));
      }
    }
    return Optional.empty();
  }

  static List<String> buildProfileHierarchy(String profile) {
    List<String> chain = new ArrayList<>(3);
    String norm = FluxFigCsvCodec.normalizeProfileHeader(profile);
    if (!norm.isEmpty()) {
      chain.add(norm);
      int colon = norm.indexOf(':');
      if (colon > 0) {
        String base = norm.substring(0, colon);
        if (!chain.contains(base)) {
          chain.add(base);
        }
      }
    }
    if (!chain.contains("DEFAULT")) {
      chain.add("DEFAULT");
    }
    return chain;
  }

  public synchronized double getDouble(String key) {
    String raw = getString(key);
    try {
      return Double.parseDouble(raw.trim());
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(
          "parameters.csv [profile '"
              + activeProfile
              + "', key '"
              + key
              + "']: invalid double value '"
              + raw
              + "'",
          e);
    }
  }

  public synchronized int getInt(String key) {
    String raw = getString(key);
    try {
      return Integer.parseInt(raw.trim());
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(
          "parameters.csv [profile '"
              + activeProfile
              + "', key '"
              + key
              + "']: invalid int value '"
              + raw
              + "'",
          e);
    }
  }

  public synchronized boolean getBoolean(String key) {
    return Boolean.parseBoolean(getString(key));
  }

  /** Returns the names of all discovered relational table tabs (without {@code .csv} suffix). */
  public synchronized Set<String> getTableNames() {
    return Collections.unmodifiableSet(relationalTables.keySet());
  }

  /**
   * Schema-free table accessor: returns all rows of any {@code <tableName>.csv} file in {@code
   * configDir} as unmodifiable {@code columnHeader -> cellValue} maps (with any active {@link
   * #setTableCellOverride} edits applied).
   */
  public synchronized List<Map<String, String>> getTable(String tableName) {
    return getTableWithCellOverrides(tableName);
  }

  List<Map<String, String>> getTableWithCellOverrides(String tableName) {
    return FluxFigRecordBinder.resolveTableWithCellOverrides(
        relationalTables, tableCellOverrides, tableName);
  }

  /**
   * Schema-free reflective record accessor (alias of {@link #bindTable(String, Class)}): maps every
   * row of {@code <tableName>.csv} to an instance of season-specific Java {@code record} type
   * {@code recordClass}.
   */
  public synchronized <T extends Record> List<T> getTable(
      String tableName, Class<T> recordClass) {
    return bindTable(tableName, recordClass);
  }

  /**
   * Schema-free reflective record accessor with profile filtering (alias of {@link
   * #bindTable(String, String, Class)}).
   */
  public synchronized <T extends Record> List<T> getTable(
      String tableName, String profile, Class<T> recordClass) {
    return bindTable(tableName, profile, recordClass);
  }

  /**
   * Returns rows of {@code tableName} resolved for {@code profile} following {@code
   * <Profile>:<Overlay> -> <Profile> -> DEFAULT} hierarchy precedence. If the table has a row-key
   * column (e.g., {@code preset}, {@code presetName}, {@code mechanism}, {@code cameraName}),
   * more-specific profile rows override less-specific {@code DEFAULT} rows with the same row key
   * without producing duplicates.
   */
  public synchronized List<Map<String, String>> getTableForProfile(
      String tableName, String profile) {
    return FluxFigRecordBinder.resolveTableForProfile(
        relationalTables, tableCellOverrides, tableName, profile);
  }

  /**
   * Explicitly registers a season-specific Java {@code record} schema for {@code tableName} so
   * {@link #validateStrict()}, {@link #persistOverridesToCsv()}, and {@link
   * #promoteSnapshotToDirectory} enforce full column, type, and enum validation on that table.
   */
  public synchronized <T extends Record> void registerTableSchema(
      String tableName, Class<T> recordClass) {
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    registeredTableSchemas.put(cleanTable, Objects.requireNonNull(recordClass));
  }

  /**
   * Schema-free reflective record binder: maps every row of {@code <tableName>.csv} to an instance
   * of season-specific Java {@code record} type {@code recordClass} by matching record component
   * names against CSV column headers. When in {@link Mode#TUNING}, automatically publishes any
   * {@link Tunable}-annotated components to {@link TuningBackend}.
   */
  public synchronized <T extends Record> List<T> bindTable(
      String tableName, Class<T> recordClass) {
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    registeredTableSchemas.put(cleanTable, Objects.requireNonNull(recordClass));
    String label = cleanTable + ".csv";
    List<Map<String, String>> rows = getTableWithCellOverrides(tableName);
    List<T> bound = FluxFigRecordBinder.bindRowsToRecord(label, rows, recordClass, false, null);
    maybePublishTunableTableRows(tableName, activeProfile, rows, recordClass);
    return bound;
  }

  /**
   * Schema-free reflective record binder with automatic {@code robot}/{@code profile} row
   * filtering. When in {@link Mode#TUNING}, automatically publishes any {@link Tunable}-annotated
   * components to {@link TuningBackend}.
   */
  public synchronized <T extends Record> List<T> bindTable(
      String tableName, String profile, Class<T> recordClass) {
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName).toLowerCase(Locale.ROOT);
    registeredTableSchemas.put(cleanTable, Objects.requireNonNull(recordClass));
    String label = cleanTable + ".csv";
    List<Map<String, String>> rows = getTableForProfile(tableName, profile);
    List<T> bound = FluxFigRecordBinder.bindRowsToRecord(label, rows, recordClass, false, null);
    maybePublishTunableTableRows(tableName, profile, rows, recordClass);
    return bound;
  }

  private <T extends Record> void maybePublishTunableTableRows(
      String tableName,
      String profile,
      List<Map<String, String>> rows,
      Class<T> recordClass) {
    if (mode != Mode.TUNING || tuningBackend == null || rows.isEmpty()) {
      return;
    }
    RecordComponent[] components = recordClass.getRecordComponents();
    List<RecordComponent> tunableComps = new ArrayList<>();
    for (RecordComponent rc : components) {
      if (rc.getAnnotation(Tunable.class) != null) {
        tunableComps.add(rc);
      }
    }
    if (tunableComps.isEmpty()) {
      return;
    }
    String cleanTable = FluxFigCsvCodec.stripCsvExtension(tableName);
    String normProf = FluxFigCsvCodec.normalizeProfileHeader(profile);
    String keyCol = FluxFigRecordBinder.findRowKeyColumnHeader(rows.get(0), recordClass);
    if (keyCol == null) {
      return;
    }
    for (Map<String, String> row : rows) {
      String rowKey = row.getOrDefault(keyCol, "").trim();
      if (rowKey.isEmpty()) {
        continue;
      }
      for (RecordComponent tc : tunableComps) {
        String compName = tc.getName();
        String val = FluxFigCsvCodec.findCellByColumnNameOrDefault(row, compName, "");
        String topic = TUNING_TOPIC_PREFIX + cleanTable + "/" + rowKey + "/" + compName;
        if (!boundTuningTopics.containsKey(topic)) {
          tuningBackend.publish(topic, val);
          boundTuningTopics.put(
              topic,
              new BoundTuningTopic(
                  topic,
                  true,
                  cleanTable.toLowerCase(Locale.ROOT),
                  normProf,
                  rowKey,
                  compName,
                  val));
        }
      }
    }
  }

  /**
   * Looks up a single typed row from {@code tableName} for the currently active profile by matching
   * {@code rowKey} (e.g., an enum constant like {@code ShotPreset.MID} or string {@code "L4"})
   * against the table's {@link RowKey} column.
   */
  public synchronized <T extends Record> Optional<T> getRowByKey(
      String tableName, Object rowKey, Class<T> recordClass) {
    return getRowByKey(tableName, activeProfile, rowKey, recordClass);
  }

  /**
   * Looks up a single typed row from {@code tableName} for {@code profile} by matching {@code
   * rowKey} against the table's {@link RowKey} column.
   */
  public synchronized <T extends Record> Optional<T> getRowByKey(
      String tableName, String profile, Object rowKey, Class<T> recordClass) {
    List<T> rows = bindTable(tableName, profile, recordClass);
    return FluxFigRecordBinder.findRowByKey(rows, rowKey, recordClass);
  }

  /**
   * Looks up a required typed row from {@code tableName} for the active profile by {@code rowKey},
   * throwing {@link IllegalArgumentException} if not found.
   */
  public synchronized <T extends Record> T getRequiredRowByKey(
      String tableName, Object rowKey, Class<T> recordClass) {
    return getRowByKey(tableName, activeProfile, rowKey, recordClass)
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Missing row key '"
                        + rowKey
                        + "' in table '"
                        + tableName
                        + "' for profile '"
                        + activeProfile
                        + "'"));
  }

  /**
   * Performs continuous piecewise linear interpolation ({@code lerp}) across {@code tableName} for
   * the currently active profile sorted in ascending order of {@code xColumn} (e.g., {@code
   * "distanceMeters"} in {@code range_table.csv}). Clamps to the minimum/maximum row when {@code
   * xValue} is outside the table's domain, and immediately reflects any live {@link Mode#TUNING}
   * cell overrides.
   */
  public synchronized <T extends Record> T interpolateTable(
      String tableName, String xColumn, double xValue, Class<T> recordClass) {
    return interpolateTable(tableName, activeProfile, xColumn, xValue, recordClass);
  }

  /**
   * Performs continuous piecewise linear interpolation ({@code lerp}) across {@code tableName} for
   * {@code profile} sorted in ascending order of {@code xColumn}.
   */
  public synchronized <T extends Record> T interpolateTable(
      String tableName,
      String profile,
      String xColumn,
      double xValue,
      Class<T> recordClass) {
    List<T> rows = bindTable(tableName, profile, recordClass);
    return FluxFigRecordBinder.interpolateRows(
        tableName, profile, rows, xColumn, xValue, recordClass);
  }

  /**
   * Resolves a {@link MechanismSpec} for the currently active profile with {@code
   * <Profile>:<Overlay> -> <Profile> -> DEFAULT} fallback inheritance and automatic CAN bus
   * resolution when switching between SystemCore ({@code can_s0}/{@code can_s2}/{@code can_s4}) and
   * a roboRIO fallback ({@code ROBORIO_CANIVORE_2026}).
   */
  public synchronized MechanismSpec getMechanismSpec(String mechanism) {
    return getMechanismSpec(activeProfile, mechanism);
  }

  /**
   * Resolves a {@link MechanismSpec} for {@code robotOrProfile} with {@code <Profile>:<Overlay> ->
   * <Profile> -> DEFAULT} fallback inheritance and automatic CAN bus resolution across profile
   * overlays (e.g., {@code COMP_2027:roborio_fallback}).
   */
  public synchronized MechanismSpec getMechanismSpec(String robotOrProfile, String mechanism) {
    for (String candidateProfile : buildProfileHierarchy(robotOrProfile)) {
      MechanismSpec spec = mechanismSpecs.get(candidateProfile + ":" + mechanism);
      if (spec != null) {
        return spec.withCanBus(resolveEffectiveCanBus(robotOrProfile, spec.canBus()));
      }
    }
    throw new IllegalArgumentException(
        "Unknown mechanism spec: " + robotOrProfile + ":" + mechanism);
  }

  String resolveEffectiveCanBus(String robotOrProfile, String rawBus) {
    if (rawBus == null || rawBus.isEmpty()) {
      return rawBus;
    }
    if (rawBus.startsWith("$")) {
      String paramKey = rawBus.substring(1).trim();
      return resolveEffectiveParameter(robotOrProfile, paramKey).orElse(rawBus);
    }

    String normBus = FluxFigCsvCodec.normalizeIdentifier(rawBus);
    String controller =
        resolveEffectiveParameter(robotOrProfile, "controllerProfile").orElse("");
    boolean isRoboRioFallback =
        controller.toUpperCase(Locale.ROOT).contains("ROBORIO")
            || FluxFigCsvCodec.normalizeProfileHeader(robotOrProfile)
                .toLowerCase(Locale.ROOT)
                .contains("roborio");

    if ("cans0".equals(normBus)) {
      Optional<String> swerveBus =
          resolveEffectiveParameter(robotOrProfile, "swerveCanBus")
              .or(() -> resolveEffectiveParameter(robotOrProfile, "driveCanBus"));
      if (swerveBus.isPresent() && !swerveBus.get().isBlank()) {
        return swerveBus.get();
      }
      if (isRoboRioFallback) {
        return "Drivetrain";
      }
    } else if ("cans2".equals(normBus)) {
      Optional<String> mechBus = resolveEffectiveParameter(robotOrProfile, "mechCanBus");
      if (mechBus.isPresent() && !mechBus.get().isBlank()) {
        return mechBus.get();
      }
      if (isRoboRioFallback) {
        return "Mech";
      }
    } else if ("cans4".equals(normBus)) {
      Optional<String> auxBus = resolveEffectiveParameter(robotOrProfile, "auxCanBus");
      if (auxBus.isPresent() && !auxBus.get().isBlank()) {
        return auxBus.get();
      }
      if (isRoboRioFallback) {
        return "rio";
      }
    }
    return rawBus;
  }

  /**
   * Returns all {@link CameraSpec} entries for the currently active profile with {@code
   * <Profile>:<Overlay> -> <Profile> -> DEFAULT} fallback inheritance.
   */
  public synchronized List<CameraSpec> getCamerasForActiveProfile() {
    return getCamerasForRobot(activeProfile);
  }

  /**
   * Returns all {@link CameraSpec} entries for {@code robotOrProfile} with {@code
   * <Profile>:<Overlay> -> <Profile> -> DEFAULT} fallback inheritance.
   */
  public synchronized List<CameraSpec> getCamerasForRobot(String robotOrProfile) {
    for (String candidateProfile : buildProfileHierarchy(robotOrProfile)) {
      List<CameraSpec> out = new ArrayList<>();
      for (CameraSpec c : cameraSpecs) {
        if (c.robot().equalsIgnoreCase(candidateProfile)) {
          out.add(c);
        }
      }
      if (!out.isEmpty()) {
        return Collections.unmodifiableList(out);
      }
    }
    return Collections.emptyList();
  }

  public synchronized Map<String, BatterySpec> getBatteries() {
    return Collections.unmodifiableMap(batterySpecs);
  }
}
