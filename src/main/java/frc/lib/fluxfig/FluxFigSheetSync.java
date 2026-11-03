package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.SheetTabFetcher;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Properties;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Package-private Google Sheets HTTP CSV export fetcher, spreadsheet URL parser, and atomic
 * {@link FluxFig#validateStrict()}-gated directory puller for {@link FluxFig}.
 */
final class FluxFigSheetSync {

  private static final Pattern GOOGLE_SHEET_URL_PATTERN =
      Pattern.compile("/spreadsheets/d/([a-zA-Z0-9_-]+)");

  private static final HttpClient HTTP_CLIENT =
      HttpClient.newBuilder()
          .followRedirects(HttpClient.Redirect.NORMAL)
          .connectTimeout(Duration.ofSeconds(3))
          .executor(
              r -> {
                Thread t = new Thread(r, "fluxfig-http");
                t.setDaemon(true);
                t.start();
              })
          .build();

  private FluxFigSheetSync() {}

  static String extractSpreadsheetId(String sheetIdOrUrl) {
    if (sheetIdOrUrl == null || sheetIdOrUrl.isBlank()) {
      throw new IllegalArgumentException("Google Sheet ID or URL must not be blank");
    }
    String trimmed = sheetIdOrUrl.trim();
    Matcher m = GOOGLE_SHEET_URL_PATTERN.matcher(trimmed);
    if (m.find()) {
      return m.group(1);
    }
    return trimmed;
  }

  static URI buildSheetCsvExportUri(String sheetIdOrUrl, String tabName) {
    String cleanTab = FluxFigCsvCodec.stripCsvExtension(tabName);
    String encodedTab = URLEncoder.encode(cleanTab, StandardCharsets.UTF_8);
    String trimmed = sheetIdOrUrl.trim();
    if ((trimmed.startsWith("http://") || trimmed.startsWith("https://"))
        && !trimmed.contains("docs.google.com/spreadsheets")) {
      String base = trimmed.endsWith("/") ? trimmed.substring(0, trimmed.length() - 1) : trimmed;
      return URI.create(base + "/" + encodedTab + ".csv");
    }
    String sheetId = extractSpreadsheetId(trimmed);
    return URI.create(
        "https://docs.google.com/spreadsheets/d/"
            + sheetId
            + "/gviz/tq?tqx=out:csv&sheet="
            + encodedTab);
  }

  static String fetchTabViaHttp(String sheetIdOrUrl, String tabName) throws IOException {
    URI uri = buildSheetCsvExportUri(sheetIdOrUrl, tabName);
    HttpRequest request =
        HttpRequest.newBuilder(uri)
            .timeout(Duration.ofSeconds(5))
            .header("Accept", "text/csv,text/plain,*/*")
            .GET()
            .build();
    try {
      HttpResponse<String> response =
          HTTP_CLIENT.send(request, HttpResponse.BodyHandlers.ofString(StandardCharsets.UTF_8));
      if (response.statusCode() != 200) {
        throw new IOException(
            "HTTP " + response.statusCode() + " fetching sheet tab '" + tabName + "' from " + uri);
      }
      String body = response.body();
      if (body == null || body.isBlank()) {
        throw new IOException("Empty CSV body returned for sheet tab '" + tabName + "'");
      }
      String trimmedLower = body.stripLeading().toLowerCase(Locale.ROOT);
      if (trimmedLower.startsWith("<!doctype html") || trimmedLower.startsWith("<html")) {
        throw new IOException(
            "Received HTML login/error page instead of CSV for sheet tab '"
                + tabName
                + "' (ensure the Google Sheet is shared as 'Anyone with the link can view')");
      }
      return body;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Interrupted while fetching sheet tab '" + tabName + "'", e);
    }
  }

  static Map<String, String> downloadSheetTabs(
      String sheetIdOrUrl, List<String> tabNames, SheetTabFetcher tabFetcher) throws IOException {
    if (tabNames == null || tabNames.isEmpty()) {
      throw new IllegalArgumentException("At least one sheet tab name must be specified");
    }
    Map<String, String> downloaded = new LinkedHashMap<>();
    for (String rawTab : tabNames) {
      String cleanTab = FluxFigCsvCodec.stripCsvExtension(rawTab);
      if (cleanTab.isEmpty()) {
        continue;
      }
      String csv = tabFetcher.fetchCsvTab(sheetIdOrUrl, cleanTab);
      downloaded.put(cleanTab + ".csv", FluxFigCsvCodec.normalizeLineEndings(csv));
    }
    return downloaded;
  }

  static List<String> discoverTabNamesForDirectory(Path configDir) throws IOException {
    if (configDir != null && Files.isDirectory(configDir)) {
      try (Stream<Path> stream = Files.list(configDir)) {
        List<String> existing =
            stream
                .filter(Files::isRegularFile)
                .map(p -> p.getFileName().toString())
                .filter(name -> name.toLowerCase(Locale.ROOT).endsWith(".csv"))
                .sorted()
                .map(FluxFigCsvCodec::stripCsvExtension)
                .toList();
        if (!existing.isEmpty()) {
          return existing;
        }
      }
    }
    return FluxFig.DEFAULT_SHEET_TABS;
  }

  static FluxFig pullFromGoogleSheet(Path configDir) throws IOException {
    Properties props = new Properties();
    Path propsFile = configDir.resolve(FluxFig.SHEET_PROPERTIES_FILE);
    if (Files.isRegularFile(propsFile)) {
      try (InputStream in = Files.newInputStream(propsFile)) {
        props.load(in);
      }
    }
    String sheetId = props.getProperty("sheetId", System.getenv("FLUXFIG_SHEET_ID"));
    if (sheetId == null || sheetId.isBlank()) {
      throw new IllegalStateException(
          "No Google Sheet ID configured in " + propsFile + " or FLUXFIG_SHEET_ID env var");
    }
    String tabsProp = props.getProperty("tabs", "");
    List<String> tabs =
        !tabsProp.isBlank()
            ? Arrays.stream(tabsProp.split(","))
                .map(String::trim)
                .filter(s -> !s.isEmpty())
                .toList()
            : discoverTabNamesForDirectory(configDir);
    return pullFromGoogleSheet(configDir, sheetId.trim(), tabs, FluxFigSheetSync::fetchTabViaHttp);
  }

  static FluxFig pullFromGoogleSheet(
      Path configDir,
      String sheetIdOrUrl,
      List<String> tabNames,
      SheetTabFetcher tabFetcher)
      throws IOException {
    List<String> effectiveTabs =
        (tabNames == null || tabNames.isEmpty())
            ? discoverTabNamesForDirectory(configDir)
            : tabNames;
    Map<String, String> downloaded = downloadSheetTabs(sheetIdOrUrl, effectiveTabs, tabFetcher);

    Map<String, String> candidateFiles = new LinkedHashMap<>();
    if (configDir != null && Files.isDirectory(configDir)) {
      try (Stream<Path> stream = Files.list(configDir)) {
        for (Path existing :
            stream
                .filter(Files::isRegularFile)
                .filter(
                    p ->
                        p.getFileName()
                            .toString()
                            .toLowerCase(Locale.ROOT)
                            .endsWith(".csv"))
                .sorted(Comparator.comparing(p -> p.getFileName().toString()))
                .toList()) {
          candidateFiles.put(
              existing.getFileName().toString(),
              Files.readString(existing, StandardCharsets.UTF_8));
        }
      }
    }
    candidateFiles.putAll(downloaded);

    FluxFig candidate = FluxFig.fromCsvMap(candidateFiles, "DEFAULT");
    List<String> errors = candidate.validateStrict();
    if (!errors.isEmpty()) {
      throw new IllegalStateException(
          "Google Sheet safety validation failed ("
              + errors.size()
              + " errors); local CSVs in "
              + configDir
              + " were not modified: "
              + errors);
    }

    Files.createDirectories(configDir);
    for (Map.Entry<String, String> entry : downloaded.entrySet()) {
      String fileName =
          entry.getKey().toLowerCase(Locale.ROOT).endsWith(".csv")
              ? entry.getKey()
              : entry.getKey() + ".csv";
      FluxFigCsvCodec.writeCsvFileAtomically(configDir, fileName, entry.getValue());
    }
    return new FluxFig(configDir);
  }
}
