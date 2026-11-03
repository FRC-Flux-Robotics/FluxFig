package frc.lib.fluxfig;

import frc.lib.fluxfig.FluxFig.ReleaseEntry;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * CLI entry point for {@link FluxFig} standalone library ({@code java -jar fluxfig.jar ...}).
 *
 * <ul>
 *   <li>{@code java -jar fluxfig.jar pull <configDir> [sheetIdOrUrl] [tab1,tab2,...]} — Downloads
 *       CSV tabs from Google Sheets (or {@code configDir/fluxfig-sheet.properties}), runs {@link
 *       FluxFig#validateStrict()}, and atomically updates {@code configDir/*.csv}.
 *   <li>{@code java -jar fluxfig.jar validate <configDir>} — Validates CSV schemas &amp; hardware
 *       safety rules ({@code <= 60 A} stator, {@code <= 40 A} supply, zero odd SystemCore CAN FD
 *       buses, zero duplicate CAN IDs across profiles) and prints {@code ConfigSHA256}, {@code
 *       EffectiveVersion}, and discovered tables.
 *   <li>{@code java -jar fluxfig.jar promote <snapshotFile> <configDir> [author] [notes]} —
 *       Harvests a tuned {@code /tmp/fluxfig-tuned-snapshot.txt} or {@code
 *       /FluxFig/EffectiveSnapshot}, applies all parameter and table-cell overrides to {@code
 *       <configDir>/*.csv} (preserving comments and extra columns), validates via {@link
 *       FluxFig#validateStrict()}, and appends the next {@code vN} version row to {@code
 *       releases.csv}.
 *   <li>{@code java -jar fluxfig.jar release <configDir> [version|auto] [author] [notes]} —
 *       Validates {@code <configDir>} and appends a frozen {@code vN} row with the current {@code
 *       ConfigSHA256} to {@code releases.csv}.
 *   <li>{@code java -jar fluxfig.jar sha256 <configDir>} — Prints the deterministic 8-char {@code
 *       ConfigSHA256}.
 *   <li>{@code java -jar fluxfig.jar get <configDir> <profile> <key>} — Resolves a parameter key
 *       with {@code <Profile>:<Overlay> -> <Profile> -> DEFAULT} inheritance.
 *   <li>{@code java -jar fluxfig.jar tables <configDir>} — Lists all discovered relational tables
 *       and their row counts.
 *   <li>{@code java -jar fluxfig.jar snapshot <configDir> [profile]} — Prints the deterministic
 *       {@code .wpilog} effective snapshot.
 * </ul>
 */
public final class FluxFigMain {

  private FluxFigMain() {}

  public static void main(String[] args) {
    if (args.length < 2) {
      System.err.println(
          "Usage: java -jar fluxfig.jar <pull|validate|promote|release|sha256|get|tables|snapshot> <configDir_or_snapshot> [args...]");
      System.exit(1);
    }
    String cmd = args[0];

    if ("promote".equals(cmd)) {
      if (args.length < 3) {
        System.err.println(
            "Usage: java -jar fluxfig.jar promote <snapshotFile> <configDir> [author] [notes]");
        System.exit(1);
      }
      Path snapshotFile = Path.of(args[1]);
      Path targetConfigDir = Path.of(args[2]);
      String author = args.length >= 4 ? args[3] : "flux-team";
      String notes = args.length >= 5 ? args[4] : "";
      try {
        String snapshotText = Files.readString(snapshotFile, StandardCharsets.UTF_8);
        ReleaseEntry rel =
            FluxFig.promoteSnapshotToDirectory(snapshotText, targetConfigDir, author, notes);
        System.out.println(
            "FluxFig PROMOTE PASSED | Version="
                + rel.version()
                + " ("
                + rel.configSha256()
                + ") | Author="
                + rel.author()
                + " | Notes="
                + rel.notes());
        return;
      } catch (IOException | RuntimeException e) {
        System.err.println("FluxFig PROMOTE FAILED: " + e.getMessage());
        System.exit(2);
      }
    }

    Path configDir = Path.of(args[1]);

    if ("pull".equals(cmd)) {
      try {
        FluxFig fig;
        if (args.length == 2) {
          fig = FluxFig.pullFromGoogleSheet(configDir);
        } else {
          String sheetIdOrUrl = args[2];
          List<String> tabs =
              args.length >= 4
                  ? Arrays.stream(args[3].split(","))
                      .map(String::trim)
                      .filter(s -> !s.isEmpty())
                      .toList()
                  : Collections.emptyList();
          fig = FluxFig.pullFromGoogleSheet(configDir, sheetIdOrUrl, tabs);
        }
        System.out.println(
            "FluxFig SHEET PULL PASSED | Version="
                + fig.getEffectiveVersion()
                + " | ConfigSHA256="
                + fig.getConfigSha256()
                + " | Tables="
                + fig.getTableNames());
        return;
      } catch (IOException | RuntimeException e) {
        System.err.println("FluxFig SHEET PULL FAILED: " + e.getMessage());
        System.exit(2);
      }
    }

    FluxFig fig = FluxFig.load(configDir);

    switch (cmd) {
      case "validate" -> {
        List<String> errors = fig.validateStrict();
        if (!errors.isEmpty()) {
          System.err.println("FluxFig VALIDATION FAILED (" + errors.size() + " errors):");
          for (String err : errors) {
            System.err.println("  - " + err);
          }
          System.exit(2);
        }
        System.out.println(
            "FluxFig VALIDATION PASSED | Version="
                + fig.getEffectiveVersion()
                + " | ConfigSHA256="
                + fig.getConfigSha256()
                + " | Tables="
                + fig.getTableNames()
                + " | Batteries="
                + fig.getBatteries().size());
      }
      case "release" -> {
        String version = args.length >= 3 ? args[2] : "auto";
        String author = args.length >= 4 ? args[3] : "flux-team";
        String notes = args.length >= 5 ? args[4] : "Published release";
        try {
          ReleaseEntry rel = fig.recordRelease(version, author, notes);
          System.out.println(
              "FluxFig RELEASE RECORDED | Version="
                  + rel.version()
                  + " ("
                  + rel.configSha256()
                  + ") | Author="
                  + rel.author()
                  + " | Notes="
                  + rel.notes());
        } catch (IOException | RuntimeException e) {
          System.err.println("FluxFig RELEASE FAILED: " + e.getMessage());
          System.exit(2);
        }
      }
      case "sha256" -> System.out.println(fig.getConfigSha256());
      case "get" -> {
        if (args.length < 4) {
          System.err.println("Usage: java -jar fluxfig.jar get <configDir> <profile> <key>");
          System.exit(1);
        }
        fig.setActiveProfile(args[2]);
        System.out.println(fig.getString(args[3]));
      }
      case "tables" -> {
        for (String table : fig.getTableNames()) {
          List<Map<String, String>> rows = fig.getTable(table);
          System.out.println(table + " (" + rows.size() + " rows)");
        }
      }
      case "snapshot" -> {
        if (args.length >= 3) {
          fig.setActiveProfile(args[2]);
        }
        System.out.print(fig.getEffectiveSnapshot());
      }
      default -> {
        System.err.println("Unknown command: " + cmd);
        System.exit(1);
      }
    }
  }
}
