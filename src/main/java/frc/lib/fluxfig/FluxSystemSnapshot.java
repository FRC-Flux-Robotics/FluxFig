package frc.lib.fluxfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Shared zero-dependency offline system telemetry snapshot ({@code system-snapshot.txt}) and
 * 14-metric SystemCore / roboRIO watch evaluator used by {@code FluxTray}, {@code FluxProcessor},
 * and the FLUX 10413 robot repositories.
 */
public record FluxSystemSnapshot(
    String controllerType,
    String hostname,
    String kernelVersion,
    double uptimeSec,
    double loopPeriodMs,
    double loopMedianMs,
    double loopP99Ms,
    double loopMaxMs,
    double firstEnablePeakMs,
    double odometryJitterMaxMs,
    double driveTraceMaxMs,
    double visionTraceMaxMs,
    double superstructureTraceMaxMs,
    double totalCpuPct,
    double javaProcessCpuPct,
    double visionDaemonCpuPct,
    double softIrqPct,
    double hardIrqPct,
    double ioWaitPct,
    long spiCanIrqCount,
    long usbXhciIrqCount,
    double cpuTempC,
    double cpuFreqMHz,
    int throttledFlags,
    double heapAllocRateMbPerSec,
    int gcCount,
    double gcTotalPauseMs,
    int gcCorrelatedLoopSpikes,
    double freeStorageMb,
    double logWriteLatencyMaxMs,
    double ipc,
    double l1dCacheMissPct,
    double llcMissPct,
    long involuntaryContextSwitches,
    List<CanInterfaceStats> canInterfaces) {

  /** Per-interface Linux SocketCAN or roboRIO CAN bus counters. */
  public record CanInterfaceStats(
      String iface,
      long rxPackets,
      long txPackets,
      long rxDropped,
      long txDropped,
      long rxOverErrors,
      int busErrorCount,
      int busRestarts,
      String canState) {}

  /** A single evaluated watch metric and its actionable engineering/pit decision. */
  public record WatchDecision(
      String category,
      String metricName,
      String measuredValue,
      String threshold,
      String severity,
      String anomalySummary,
      String actionableDecision) {}

  /**
   * Single non-intrusive BusyBox-safe shell command executed over SSH after a run to capture Linux
   * {@code /proc} and {@code /sys} counters on SystemCore or roboRIO without touching robot Java
   * code.
   */
  public static final String REMOTE_CAPTURE_SH_COMMAND =
      "echo '[PLATFORM]'; "
          + "if [ -d /home/systemcore ] || ip link show can_s0 >/dev/null 2>&1; then echo 'controllerType=SYSTEMCORE'; "
          + "elif [ -d /home/lvuser ] || [ -f /etc/natinst/share/ni-rt.ini ]; then echo 'controllerType=ROBORIO'; "
          + "else echo 'controllerType=UNKNOWN'; fi; "
          + "echo \"hostname=$(hostname 2>/dev/null || echo 'NOT MEASURED')\"; "
          + "echo \"kernelVersion=$(uname -r 2>/dev/null || echo 'NOT MEASURED')\"; "
          + "awk '{print \"uptimeSec=\"$1}' /proc/uptime 2>/dev/null || true; "
          + "echo '[CPU_AND_IRQ]'; "
          + "awk '/^cpu / {total=$2+$3+$4+$5+$6+$7+$8; if (total>0) {printf \"totalCpuPct=%.1f\\nioWaitPct=%.1f\\nhardIrqPct=%.1f\\nsoftIrqPct=%.1f\\n\", (total-$5)*100.0/total, $6*100.0/total, $7*100.0/total, $8*100.0/total}}' /proc/stat 2>/dev/null || true; "
          + "if ps -eo pcpu,comm >/dev/null 2>&1; then ps -eo pcpu,comm 2>/dev/null | awk '$1 ~ /^[0-9]+(\\.[0-9]+)?$/ {if ($2 ~ /java/) j+=$1; if ($2 ~ /limelight|photon/) v+=$1} END {if (j>0) printf \"javaProcessCpuPct=%.1f\\n\", j; if (v>0) printf \"visionDaemonCpuPct=%.1f\\n\", v}'; fi; "
          + "echo '[THERMAL_AND_POWER]'; "
          + "if [ -f /sys/class/thermal/thermal_zone0/temp ]; then awk '{printf \"cpuTempC=%.1f\\n\", $1/1000.0}' /sys/class/thermal/thermal_zone0/temp; fi; "
          + "if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]; then awk '{printf \"cpuFreqMHz=%.0f\\n\", $1/1000.0}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq; fi; "
          + "if command -v vcgencmd >/dev/null 2>&1; then vcgencmd get_throttled 2>/dev/null | sed 's/throttled=/throttledHex=/' || true; fi; "
          + "echo '[SOCKETCAN]'; "
          + "for d in /sys/class/net/can_*; do "
          + "  [ -e \"$d\" ] || continue; "
          + "  b=$(basename \"$d\"); "
          + "  rxp=$(cat \"$d/statistics/rx_packets\" 2>/dev/null || echo 0); "
          + "  txp=$(cat \"$d/statistics/tx_packets\" 2>/dev/null || echo 0); "
          + "  rxd=$(cat \"$d/statistics/rx_dropped\" 2>/dev/null || echo 0); "
          + "  txd=$(cat \"$d/statistics/tx_dropped\" 2>/dev/null || echo 0); "
          + "  rxo=$(cat \"$d/statistics/rx_over_errors\" 2>/dev/null || echo 0); "
          + "  ipout=$(ip -d -s link show \"$b\" 2>/dev/null || true); "
          + "  st=$(printf '%s\\n' \"$ipout\" | awk '/can state/ {for(i=1;i<=NF;i++) if($i==\"state\") {print $(i+1); exit}}'); "
          + "  berr=$(printf '%s\\n' \"$ipout\" | awk '/berr-counter/ {tx=0; rx=0; for(i=1;i<=NF;i++) {if($i==\"tx\") tx=$(i+1)+0; if($i==\"rx\") rx=$(i+1)+0} print tx+rx; exit}'); "
          + "  rst=$(printf '%s\\n' \"$ipout\" | awk '/restarts/ {getline; print $6+0; exit}'); "
          + "  echo \"can.${b}=${rxp},${txp},${rxd},${txd},${rxo},${berr:-0},${rst:-0},${st:-UNKNOWN}\"; "
          + "done; "
          + "echo '[STORAGE]'; "
          + "for d in /U/logs /home/systemcore /home/lvuser /; do "
          + "  if [ -d \"$d\" ]; then "
          + "    df -k \"$d\" 2>/dev/null | awk 'NR>1 && NF>=4 {avail=$(NF-2)} END {if (avail ~ /^[0-9]+$/) printf \"freeStorageMb=%.1f\\n\", avail/1024.0}'; "
          + "    break; "
          + "  fi; "
          + "done";

  /** Creates a nominal healthy SystemCore baseline snapshot. */
  public static FluxSystemSnapshot healthySystemCoreBaseline() {
    List<CanInterfaceStats> cans =
        List.of(
            new CanInterfaceStats("can_s0", 385_000L, 92_000L, 0L, 0L, 0L, 0, 0, "ERROR-ACTIVE"),
            new CanInterfaceStats("can_s1", 0L, 0L, 0L, 0L, 0L, 0, 0, "ERROR-ACTIVE"),
            new CanInterfaceStats("can_s2", 142_000L, 48_000L, 0L, 0L, 0L, 0, 0, "ERROR-ACTIVE"),
            new CanInterfaceStats("can_s3", 0L, 0L, 0L, 0L, 0L, 0, 0, "ERROR-ACTIVE"),
            new CanInterfaceStats("can_s4", 18_500L, 4_200L, 0L, 0L, 0L, 0, 0, "ERROR-ACTIVE"));
    return new FluxSystemSnapshot(
        "SYSTEMCORE",
        "SystemCore-10413",
        "6.6.47-rt39-v8-16k+",
        482.5,
        5.0,
        1.48,
        2.85,
        4.10,
        3.95,
        0.38,
        0.52,
        0.61,
        0.24,
        42.0,
        11.5,
        26.0,
        4.5,
        1.2,
        0.4,
        41_200L,
        28_400L,
        58.4,
        2400.0,
        0x0,
        0.18,
        2,
        0.9,
        0,
        1180.0,
        2.1,
        1.42,
        1.8,
        3.2,
        18L,
        cans);
  }

  /**
   * Serializes this snapshot into the canonical {@code system-snapshot.txt} format, omitting
   * unmeasured ({@code NaN} / negative counter) fields so absent metrics are never fabricated.
   */
  public String serialize() {
    StringBuilder sb = new StringBuilder(1024);
    sb.append("# FLUX 10413 Offline System & Controller Snapshot\n");
    sb.append("[PLATFORM]\n");
    appendStringLine(sb, "controllerType", controllerType);
    appendStringLine(sb, "hostname", hostname);
    appendStringLine(sb, "kernelVersion", kernelVersion);
    appendDoubleLine(sb, "uptimeSec", "%.1f", uptimeSec);

    sb.append("\n[LOOP_AND_TIMING]\n");
    appendDoubleLine(sb, "loopPeriodMs", "%.2f", loopPeriodMs);
    appendDoubleLine(sb, "loopMedianMs", "%.2f", loopMedianMs);
    appendDoubleLine(sb, "loopP99Ms", "%.2f", loopP99Ms);
    appendDoubleLine(sb, "loopMaxMs", "%.2f", loopMaxMs);
    appendDoubleLine(sb, "firstEnablePeakMs", "%.2f", firstEnablePeakMs);
    appendDoubleLine(sb, "odometryJitterMaxMs", "%.2f", odometryJitterMaxMs);
    appendDoubleLine(sb, "driveTraceMaxMs", "%.2f", driveTraceMaxMs);
    appendDoubleLine(sb, "visionTraceMaxMs", "%.2f", visionTraceMaxMs);
    appendDoubleLine(sb, "superstructureTraceMaxMs", "%.2f", superstructureTraceMaxMs);

    sb.append("\n[CPU_AND_IRQ]\n");
    appendDoubleLine(sb, "totalCpuPct", "%.1f", totalCpuPct);
    appendDoubleLine(sb, "javaProcessCpuPct", "%.1f", javaProcessCpuPct);
    appendDoubleLine(sb, "visionDaemonCpuPct", "%.1f", visionDaemonCpuPct);
    appendDoubleLine(sb, "softIrqPct", "%.1f", softIrqPct);
    appendDoubleLine(sb, "hardIrqPct", "%.1f", hardIrqPct);
    appendDoubleLine(sb, "ioWaitPct", "%.1f", ioWaitPct);
    appendLongLine(sb, "spiCanIrqCount", spiCanIrqCount);
    appendLongLine(sb, "usbXhciIrqCount", usbXhciIrqCount);

    sb.append("\n[THERMAL_AND_POWER]\n");
    appendDoubleLine(sb, "cpuTempC", "%.1f", cpuTempC);
    appendDoubleLine(sb, "cpuFreqMHz", "%.0f", cpuFreqMHz);
    if (throttledFlags >= 0) {
      sb.append(String.format(Locale.US, "throttledHex=0x%x\n", throttledFlags));
    }

    sb.append("\n[SOCKETCAN]\n");
    for (CanInterfaceStats c : canInterfaces) {
      sb.append("can.")
          .append(c.iface())
          .append('=')
          .append(c.rxPackets())
          .append(',')
          .append(c.txPackets())
          .append(',')
          .append(c.rxDropped())
          .append(',')
          .append(c.txDropped())
          .append(',')
          .append(c.rxOverErrors())
          .append(',')
          .append(c.busErrorCount())
          .append(',')
          .append(c.busRestarts())
          .append(',')
          .append(c.canState())
          .append('\n');
    }

    sb.append("\n[JVM_AND_STORAGE]\n");
    appendDoubleLine(sb, "heapAllocRateMbPerSec", "%.2f", heapAllocRateMbPerSec);
    appendLongLine(sb, "gcCount", gcCount);
    appendDoubleLine(sb, "gcTotalPauseMs", "%.2f", gcTotalPauseMs);
    appendLongLine(sb, "gcCorrelatedLoopSpikes", gcCorrelatedLoopSpikes);
    appendDoubleLine(sb, "freeStorageMb", "%.1f", freeStorageMb);
    appendDoubleLine(sb, "logWriteLatencyMaxMs", "%.2f", logWriteLatencyMaxMs);

    sb.append("\n[HARDWARE_PMU]\n");
    appendDoubleLine(sb, "ipc", "%.2f", ipc);
    appendDoubleLine(sb, "l1dCacheMissPct", "%.2f", l1dCacheMissPct);
    appendDoubleLine(sb, "llcMissPct", "%.2f", llcMissPct);
    appendLongLine(sb, "involuntaryContextSwitches", involuntaryContextSwitches);
    return sb.toString();
  }

  /**
   * Parses {@code system-snapshot.txt} text into a structured {@link FluxSystemSnapshot}. Absent
   * numeric metrics return {@link Double#NaN} (or {@code -1} for integer counters) and absent
   * strings return {@code "NOT MEASURED"} (or {@code "UNKNOWN"} for {@code controllerType}).
   */
  public static FluxSystemSnapshot parse(String rawText) {
    Map<String, String> kv = new LinkedHashMap<>();
    List<CanInterfaceStats> cans = new ArrayList<>();
    if (rawText != null) {
      for (String rawLine : rawText.split("\\R")) {
        String line = rawLine.trim();
        if (line.isEmpty() || line.startsWith("#") || line.startsWith("[")) {
          continue;
        }
        int eq = line.indexOf('=');
        if (eq <= 0) {
          continue;
        }
        String k = line.substring(0, eq).trim();
        String v = line.substring(eq + 1).trim();
        if (k.startsWith("can.")) {
          String iface = k.substring(4);
          String[] p = v.split(",", -1);
          if (p.length >= 8) {
            cans.add(
                new CanInterfaceStats(
                    iface,
                    parseLong(p[0], 0L),
                    parseLong(p[1], 0L),
                    parseLong(p[2], 0L),
                    parseLong(p[3], 0L),
                    parseLong(p[4], 0L),
                    (int) parseLong(p[5], 0L),
                    (int) parseLong(p[6], 0L),
                    p[7].trim()));
          }
        } else {
          kv.put(k, v);
        }
      }
    }

    String controllerType = nonBlankOrDefault(kv.get("controllerType"), "UNKNOWN");

    return new FluxSystemSnapshot(
        controllerType,
        nonBlankOrDefault(kv.get("hostname"), "NOT MEASURED"),
        nonBlankOrDefault(kv.get("kernelVersion"), "NOT MEASURED"),
        parseDoubleOrNaN(kv.get("uptimeSec")),
        parseDoubleOrNaN(kv.get("loopPeriodMs")),
        parseDoubleOrNaN(kv.get("loopMedianMs")),
        parseDoubleOrNaN(kv.get("loopP99Ms")),
        parseDoubleOrNaN(kv.get("loopMaxMs")),
        parseDoubleOrNaN(kv.get("firstEnablePeakMs")),
        parseDoubleOrNaN(kv.get("odometryJitterMaxMs")),
        parseDoubleOrNaN(kv.get("driveTraceMaxMs")),
        parseDoubleOrNaN(kv.get("visionTraceMaxMs")),
        parseDoubleOrNaN(kv.get("superstructureTraceMaxMs")),
        parseDoubleOrNaN(kv.get("totalCpuPct")),
        parseDoubleOrNaN(kv.get("javaProcessCpuPct")),
        parseDoubleOrNaN(kv.get("visionDaemonCpuPct")),
        parseDoubleOrNaN(kv.get("softIrqPct")),
        parseDoubleOrNaN(kv.get("hardIrqPct")),
        parseDoubleOrNaN(kv.getOrDefault("ioWaitPct", kv.get("iowaitPct"))),
        parseLong(kv.get("spiCanIrqCount"), -1L),
        parseLong(kv.get("usbXhciIrqCount"), -1L),
        parseDoubleOrNaN(kv.get("cpuTempC")),
        parseDoubleOrNaN(kv.get("cpuFreqMHz")),
        parseHexOrInt(kv.get("throttledHex"), -1),
        parseDoubleOrNaN(kv.get("heapAllocRateMbPerSec")),
        (int) parseLong(kv.get("gcCount"), -1L),
        parseDoubleOrNaN(kv.get("gcTotalPauseMs")),
        (int) parseLong(kv.get("gcCorrelatedLoopSpikes"), -1L),
        parseDoubleOrNaN(kv.get("freeStorageMb")),
        parseDoubleOrNaN(kv.get("logWriteLatencyMaxMs")),
        parseDoubleOrNaN(kv.get("ipc")),
        parseDoubleOrNaN(kv.get("l1dCacheMissPct")),
        parseDoubleOrNaN(kv.get("llcMissPct")),
        parseLong(kv.get("involuntaryContextSwitches"), -1L),
        Collections.unmodifiableList(cans));
  }

  /** Formats a double metric or returns {@code "NOT MEASURED"} when {@code NaN}/infinite. */
  public static String formatMetric(double value, String formatPattern, String suffix) {
    if (!Double.isFinite(value)) {
      return "NOT MEASURED";
    }
    return String.format(Locale.US, formatPattern, value) + (suffix != null ? suffix : "");
  }

  /** Formats a non-negative integer counter or returns {@code "NOT MEASURED"} when absent. */
  public static String formatCounter(long value, String suffix) {
    if (value < 0) {
      return "NOT MEASURED";
    }
    return value + (suffix != null ? suffix : "");
  }

  /** Formats a non-negative hex bitmask or returns {@code "NOT MEASURED"} when absent. */
  public static String formatHexFlags(int value) {
    if (value < 0) {
      return "NOT MEASURED";
    }
    return String.format(Locale.US, "0x%x", value);
  }

  /** Evaluates all 14 offline watch metrics on this snapshot. */
  public List<WatchDecision> evaluate() {
    return evaluate(this, true, Double.isFinite(freeStorageMb));
  }

  /**
   * Evaluates all 14 offline watch metrics across 6 categories and maps every detected anomaly to a
   * concrete engineering, {@code FluxFig} CSV, or pit maintenance decision. Unmeasured ({@code
   * NaN} / negative counter) fields are skipped honestly rather than triggering false alerts.
   *
   * @param s parsed or synthesized {@link FluxSystemSnapshot}
   * @param hasSnapshotFile whether a physical {@code system-snapshot.txt} file was present
   * @param evaluateStorage whether {@code freeStorageMb} was explicitly recorded in the snapshot
   */
  public static List<WatchDecision> evaluate(
      FluxSystemSnapshot s, boolean hasSnapshotFile, boolean evaluateStorage) {
    List<WatchDecision> out = new ArrayList<>();
    boolean isRoboRio =
        s.controllerType() != null
            && s.controllerType().toUpperCase(Locale.ROOT).contains("ROBORIO");
    double effectivePeriodMs =
        Double.isFinite(s.loopPeriodMs()) && s.loopPeriodMs() > 0.0
            ? s.loopPeriodMs()
            : (isRoboRio ? 20.0 : 5.0);

    double p99LimitMs = effectivePeriodMs * 1.30; // 6.5 ms @ 200 Hz, 26.0 ms @ 50 Hz
    if (Double.isFinite(s.loopP99Ms()) && s.loopP99Ms() > p99LimitMs) {
      String loopDecision =
          (isRoboRio || effectivePeriodMs >= 15.0)
              ? "Decision: Profile subsystem periodic() traces in AdvantageScope and move blocking CAN/vision work off the 50 Hz roboRIO main thread."
              : "Decision: Change loopPeriodMs in parameters.csv (FluxFig) from 5 ms (200 Hz) to 10 ms (100 Hz) so ChassisSpeeds.discretize() and PID dt stay deterministic.";
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "Loop Period Jitter (p99 Δt)",
              String.format(
                  Locale.US,
                  "p99=%s (max=%s)",
                  formatMetric(s.loopP99Ms(), "%.2f", " ms"),
                  formatMetric(s.loopMaxMs(), "%.2f", " ms")),
              String.format(Locale.US, "<= %.2f ms", p99LimitMs),
              s.loopP99Ms() > effectivePeriodMs * 1.75 ? "CRITICAL" : "WARN",
              String.format(
                  Locale.US,
                  "Control loop p99 (%.2f ms) exceeded %.0f Hz budget (%.1f ms period).",
                  s.loopP99Ms(),
                  1000.0 / effectivePeriodMs,
                  effectivePeriodMs),
              loopDecision));
    }

    boolean visionHigh = Double.isFinite(s.visionTraceMaxMs()) && s.visionTraceMaxMs() > 2.5;
    boolean driveHigh = Double.isFinite(s.driveTraceMaxMs()) && s.driveTraceMaxMs() > 2.0;
    boolean superHigh =
        Double.isFinite(s.superstructureTraceMaxMs()) && s.superstructureTraceMaxMs() > 1.5;
    if (visionHigh || driveHigh || superHigh) {
      double vVal = Double.isFinite(s.visionTraceMaxMs()) ? s.visionTraceMaxMs() : -1.0;
      double dVal = Double.isFinite(s.driveTraceMaxMs()) ? s.driveTraceMaxMs() : -1.0;
      double suVal =
          Double.isFinite(s.superstructureTraceMaxMs()) ? s.superstructureTraceMaxMs() : -1.0;
      String culprit =
          vVal >= dVal && vVal >= suVal
              ? "VisionMs"
              : (dVal >= suVal ? "DriveMs" : "SuperstructureMs");
      double peak = Math.max(vVal, Math.max(dVal, suVal));
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "Per-Subsystem Execution Breakdown (" + culprit + ")",
              String.format(
                  Locale.US,
                  "Drive=%s, Vision=%s, Super=%s",
                  formatMetric(s.driveTraceMaxMs(), "%.2f", " ms"),
                  formatMetric(s.visionTraceMaxMs(), "%.2f", " ms"),
                  formatMetric(s.superstructureTraceMaxMs(), "%.2f", " ms")),
              "Drive <= 2.0 ms, Vision <= 2.5 ms, Super <= 1.5 ms",
              peak > 4.0 ? "CRITICAL" : "WARN",
              culprit
                  + " spiked to "
                  + String.format(Locale.US, "%.2f ms", peak)
                  + " in LoggedTracer.",
              "VisionMs".equals(culprit)
                  ? "Decision: Cap per-loop camera queue drain or tighten AprilTag max-distance/ambiguity rejection in Vision.periodic()."
                  : "Decision: Move synchronous CTRE configurator .apply() calls out of periodic() into init or async background worker."));
    }

    if (Double.isFinite(s.firstEnablePeakMs())
        && Double.isFinite(s.loopMedianMs())
        && s.firstEnablePeakMs() > Math.max(12.0, effectivePeriodMs * 2.4)
        && s.loopMedianMs() > 0.0
        && s.loopMedianMs() < effectivePeriodMs * 0.7) {
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "First-Enable Cold JIT Spike (t=0..0.5s)",
              String.format(
                  Locale.US,
                  "FirstEnable=%.2f ms vs Median=%.2f ms",
                  s.firstEnablePeakMs(),
                  s.loopMedianMs()),
              "<= 12.0 ms",
              "WARN",
              "First Autonomous loop spiked due to cold JVM C2 JIT compilation.",
              "Decision: Add dummy warmup iterations for PathPlanner trajectory follower, ShotCalculator, and ZeroPriorBootstrapSolver inside robotInit()."));
    }

    if (Double.isFinite(s.odometryJitterMaxMs()) && s.odometryJitterMaxMs() > 1.5) {
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "250 Hz Odometry Thread Jitter",
              String.format(Locale.US, "±%.2f ms around 4.0 ms", s.odometryJitterMaxMs()),
              "<= ±1.50 ms",
              s.odometryJitterMaxMs() > 2.5 ? "CRITICAL" : "WARN",
              "250 Hz PhoenixOdometryThread experienced scheduling jitter.",
              isRoboRio
                  ? "Decision: Verify CANivore USB bandwidth and lower non-critical TalonFX status frame rates on Drivetrain bus."
                  : "Decision: Pin PhoenixOdometryThread to Core 3 (taskset -c 3) away from Linux USB/Network interrupt handlers on Core 0."));
    }

    for (CanInterfaceStats can : s.canInterfaces()) {
      boolean isOddSpiSharedPort =
          "can_s1".equalsIgnoreCase(can.iface()) || "can_s3".equalsIgnoreCase(can.iface());
      if (isOddSpiSharedPort && (can.rxPackets() > 0 || can.txPackets() > 0)) {
        out.add(
            new WatchDecision(
                "2. SocketCAN & SPI Bus",
                "Forbidden Odd CAN FD Port Traffic (" + can.iface() + ")",
                String.format(
                    Locale.US,
                    "rx=%d, tx=%d on %s",
                    can.rxPackets(),
                    can.txPackets(),
                    can.iface()),
                "0 packets on can_s1 / can_s3",
                "CRITICAL",
                can.iface()
                    + " shares an internal 12,500 FPS SPI bus with an even port (SystemcoreTesting Issue #342).",
                "Decision: Move CAN harness from "
                    + can.iface()
                    + " to even port can_s0, can_s2, or can_s4 and update parameters.csv / mechanisms.csv."));
      }

      if (can.rxDropped() > 0 || can.rxOverErrors() > 0 || can.txDropped() > 0) {
        out.add(
            new WatchDecision(
                "2. SocketCAN & SPI Bus",
                "SocketCAN Drops / Overruns (" + can.iface() + ")",
                String.format(
                    Locale.US,
                    "rxDropped=%d, rxOver=%d, txDropped=%d",
                    can.rxDropped(),
                    can.rxOverErrors(),
                    can.txDropped()),
                "0 drops / overruns",
                can.rxOverErrors() > 10 || can.rxDropped() > 10 ? "CRITICAL" : "WARN",
                "MCP2518FD SPI RX/TX buffer overflowed on " + can.iface() + ".",
                "Decision: Reduce non-essential TalonFX status signal frequencies (e.g., motor temperature 50 Hz -> 4 Hz) in PhoenixSignals and verify CAN 1 / CAN 3 are empty."));
      }

      boolean badState =
          !"ERROR-ACTIVE".equalsIgnoreCase(can.canState())
              && !"UNKNOWN".equalsIgnoreCase(can.canState())
              && !"NOT MEASURED".equalsIgnoreCase(can.canState());
      if (badState || can.busErrorCount() > 0 || can.busRestarts() > 0) {
        out.add(
            new WatchDecision(
                "2. SocketCAN & SPI Bus",
                "CAN Error State / Bus-Off (" + can.iface() + ")",
                String.format(
                    Locale.US,
                    "state=%s, berr=%d, restarts=%d",
                    can.canState(),
                    can.busErrorCount(),
                    can.busRestarts()),
                "ERROR-ACTIVE, berr=0",
                "BUS-OFF".equalsIgnoreCase(can.canState()) || can.busRestarts() > 0 || badState
                    ? "CRITICAL"
                    : "WARN",
                "Physical CAN FD frame errors or bus-off transition detected on "
                    + can.iface()
                    + ".",
                "Decision: Pit Electrical Work Order — inspect 120-Ohm termination resistors, Weidmuller/Anderson CAN splices, and frayed twisted-pair stubs on "
                    + can.iface()
                    + " before next match (Issue #163)."));
      }
    }

    if (Double.isFinite(s.softIrqPct()) && s.softIrqPct() > 25.0) {
      out.add(
          new WatchDecision(
              "2. SocketCAN & SPI Bus",
              "Kernel softirq & SPI Interrupt Load",
              String.format(
                  Locale.US,
                  "softirq=%s, hardirq=%s",
                  formatMetric(s.softIrqPct(), "%.1f", "%"),
                  formatMetric(s.hardIrqPct(), "%.1f", "%")),
              "softirq <= 25.0%",
              s.softIrqPct() > 35.0 ? "CRITICAL" : "WARN",
              "Linux kernel softirq processing (SPI SocketCAN + USB UVC cameras) is saturating CPU0.",
              "Decision: Balance CAN status frame rates or lower OV9281 USB camera capture rate from 90 FPS -> 60 FPS."));
    }

    if (Double.isFinite(s.totalCpuPct())
        && Double.isFinite(s.javaProcessCpuPct())
        && s.totalCpuPct() > 80.0
        && s.javaProcessCpuPct() < 30.0) {
      out.add(
          new WatchDecision(
              "3. CPU, Vision & Thermal",
              "Java CPU % vs. Native Vision Daemon CPU %",
              String.format(
                  Locale.US,
                  "Total=%s (Java=%s, Vision C++=%s)",
                  formatMetric(s.totalCpuPct(), "%.1f", "%"),
                  formatMetric(s.javaProcessCpuPct(), "%.1f", "%"),
                  formatMetric(s.visionDaemonCpuPct(), "%.1f", "%")),
              "Total CPU <= 80.0%",
              "WARN",
              "Native C++ limelightsc0..3 AprilTag daemons (not Java robot code) are saturating the Cortex-A76 cores.",
              "Decision: Enable 2x detector decimation during Teleop or reduce camera FPS in cameras.csv."));
    }

    boolean hasThrottled = s.throttledFlags() >= 0;
    boolean thermalThrottled = hasThrottled && (s.throttledFlags() & 0x40004) != 0;
    boolean freqThrottled =
        hasSnapshotFile
            && "SYSTEMCORE".equalsIgnoreCase(s.controllerType())
            && Double.isFinite(s.cpuFreqMHz())
            && s.cpuFreqMHz() > 0.0
            && s.cpuFreqMHz() < 2000.0;
    boolean tempHigh = Double.isFinite(s.cpuTempC()) && s.cpuTempC() >= 80.0;
    if (tempHigh || thermalThrottled || freqThrottled) {
      out.add(
          new WatchDecision(
              "3. CPU, Vision & Thermal",
              "CPU Die Temperature & CM5 Thermal Throttling",
              String.format(
                  Locale.US,
                  "%s @ %s (throttled=%s)",
                  formatMetric(s.cpuTempC(), "%.1f", " °C"),
                  formatMetric(s.cpuFreqMHz(), "%.0f", " MHz"),
                  formatHexFlags(s.throttledFlags())),
              "< 80.0 °C, 2400 MHz, 0x0",
              (Double.isFinite(s.cpuTempC()) && s.cpuTempC() >= 85.0)
                      || (hasThrottled && (s.throttledFlags() & 0x4) != 0)
                  ? "CRITICAL"
                  : "WARN",
              "SoC reached thermal throttle threshold inside electronics enclosure.",
              "Decision: Add a cooling fan over controller heatsink or throttle idle camera processing while Disabled after PreMatchCalibrator locks."));
    }

    boolean underVoltage = hasThrottled && (s.throttledFlags() & 0x10001) != 0;
    if (underVoltage) {
      out.add(
          new WatchDecision(
              "3. CPU, Vision & Thermal",
              "CM5 5V Rail Undervoltage Sticky Bit",
              String.format(Locale.US, "throttled=0x%x (5V droop bit set)", s.throttledFlags()),
              "0x0 (No 5V rail droop)",
              (s.throttledFlags() & 0x1) != 0 ? "CRITICAL" : "WARN",
              "SystemCore internal 5V rail drooped below threshold (distinct from 12V battery sag).",
              "Decision: Check USB 3.0 camera 5V current draw and 3.3V/5V SmartIO sensors for shorts or regulator overload."));
    }

    if (Double.isFinite(s.heapAllocRateMbPerSec()) && s.heapAllocRateMbPerSec() > 2.0) {
      out.add(
          new WatchDecision(
              "4. JVM Memory & GC",
              "Heap Sawtooth Allocation Rate",
              String.format(Locale.US, "%.2f MB/s", s.heapAllocRateMbPerSec()),
              "<= 2.00 MB/s",
              s.heapAllocRateMbPerSec() > 5.0 ? "CRITICAL" : "WARN",
              "High per-loop object allocation rate while Enabled causing frequent GC sawtooth cycles.",
              "Decision: Inspect recent student commands for per-loop Stream, Optional, String concatenation, or array allocations in execute()/periodic()."));
    }

    if (s.gcCorrelatedLoopSpikes() > 0) {
      String gcDecision =
          isRoboRio
              ? "Decision: Keep -XX:+UseSerialGC on 32-bit roboRIO 2.0 (never enable 64-bit ZGC or -Xms512m) and eliminate per-loop object allocations in periodic()."
              : "Decision: Update JVM launch flags in build.gradle to -XX:+UseZGC -XX:+AlwaysPreTouch -Xms512m -Xmx512m.";
      out.add(
          new WatchDecision(
              "4. JVM Memory & GC",
              "GC Pause <-> Loop Overrun Correlation",
              String.format(
                  Locale.US,
                  "%d loop overruns aligned with GC (%s GCs, %s total)",
                  s.gcCorrelatedLoopSpikes(),
                  formatCounter(s.gcCount(), ""),
                  formatMetric(s.gcTotalPauseMs(), "%.1f", " ms")),
              "0 GC-induced loop overruns",
              s.gcCorrelatedLoopSpikes() >= 3 ? "CRITICAL" : "WARN",
              "JVM Garbage Collection pauses directly caused control loop overruns.",
              gcDecision));
    }

    boolean lowFreeStorage =
        evaluateStorage && Double.isFinite(s.freeStorageMb()) && s.freeStorageMb() < 400.0;
    boolean highIoWait = Double.isFinite(s.ioWaitPct()) && s.ioWaitPct() > 5.0;
    boolean highLogLatency =
        Double.isFinite(s.logWriteLatencyMaxMs()) && s.logWriteLatencyMaxMs() > 25.0;
    if (hasSnapshotFile && (lowFreeStorage || highIoWait || highLogLatency)) {
      out.add(
          new WatchDecision(
              "5. Storage & eMMC I/O",
              "eMMC Free Space & Log I/O Wait",
              String.format(
                  Locale.US,
                  "Free=%s, iowait=%s, writeMax=%s",
                  formatMetric(s.freeStorageMb(), "%.0f", " MB"),
                  formatMetric(s.ioWaitPct(), "%.1f", "%"),
                  formatMetric(s.logWriteLatencyMaxMs(), "%.1f", " ms")),
              "Free >= 400 MB, iowait <= 5.0%",
              (evaluateStorage && Double.isFinite(s.freeStorageMb()) && s.freeStorageMb() < 150.0)
                  ? "CRITICAL"
                  : "WARN",
              "Storage pressure or disk write latency spikes detected during match logging.",
              "Decision: Clean old robot logs on /home/lvuser/logs or /U/logs and verify SignalLogger.enableAutoLogging(false) is active."));
    }

    boolean lowIpc = Double.isFinite(s.ipc()) && s.ipc() > 0.0 && s.ipc() < 0.80;
    boolean highL1d = Double.isFinite(s.l1dCacheMissPct()) && s.l1dCacheMissPct() > 8.0;
    boolean highLlc = Double.isFinite(s.llcMissPct()) && s.llcMissPct() > 15.0;
    boolean highCtx = s.involuntaryContextSwitches() > 500L;
    if (hasSnapshotFile && (lowIpc || highL1d || highLlc || highCtx)) {
      out.add(
          new WatchDecision(
              "6. ARMv8 Hardware PMU",
              "Cortex-A76 IPC, L3 Cache Misses & Context Switches",
              String.format(
                  Locale.US,
                  "IPC=%s, L1D Miss=%s, LLC Miss=%s, InvolCtxSw=%s",
                  formatMetric(s.ipc(), "%.2f", ""),
                  formatMetric(s.l1dCacheMissPct(), "%.1f", "%"),
                  formatMetric(s.llcMissPct(), "%.1f", "%"),
                  formatCounter(s.involuntaryContextSwitches(), "")),
              "IPC >= 0.80, L1D <= 8.0%, LLC Miss <= 15.0%, InvolCtxSw <= 500",
              "WARN",
              "4x OV9281 camera pipelines are thrashing the shared 2 MB L3 cache or preempting real-time threads.",
              "Decision: Isolate Core 3 (isolcpus=3 / taskset) for the Java robot process + 250 Hz odometry thread and pin limelightsc0..3 daemons to Cores 0-2."));
    }

    return Collections.unmodifiableList(out);
  }

  private static void appendStringLine(StringBuilder sb, String key, String value) {
    if (value != null && !value.isBlank() && !"NOT MEASURED".equalsIgnoreCase(value.trim())) {
      sb.append(key).append('=').append(value).append('\n');
    }
  }

  private static void appendDoubleLine(
      StringBuilder sb, String key, String formatPattern, double value) {
    if (Double.isFinite(value)) {
      sb.append(key).append('=').append(String.format(Locale.US, formatPattern, value)).append('\n');
    }
  }

  private static void appendLongLine(StringBuilder sb, String key, long value) {
    if (value >= 0L) {
      sb.append(key).append('=').append(value).append('\n');
    }
  }

  private static String nonBlankOrDefault(String s, String def) {
    if (s == null || s.isBlank()) {
      return def;
    }
    return s.trim();
  }

  private static double parseDoubleOrNaN(String s) {
    if (s == null || s.isBlank()) {
      return Double.NaN;
    }
    String clean = s.trim();
    if ("NOT MEASURED".equalsIgnoreCase(clean) || "NaN".equalsIgnoreCase(clean)) {
      return Double.NaN;
    }
    try {
      double parsed = Double.parseDouble(clean);
      return Double.isFinite(parsed) ? parsed : Double.NaN;
    } catch (NumberFormatException e) {
      return Double.NaN;
    }
  }

  private static long parseLong(String s, long def) {
    if (s == null || s.isBlank()) {
      return def;
    }
    String clean = s.trim();
    if ("NOT MEASURED".equalsIgnoreCase(clean)) {
      return def;
    }
    try {
      return Long.parseLong(clean);
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static int parseHexOrInt(String s, int def) {
    if (s == null || s.isBlank()) {
      return def;
    }
    String clean = s.trim().toLowerCase(Locale.ROOT);
    if ("not measured".equals(clean)) {
      return def;
    }
    try {
      if (clean.startsWith("0x")) {
        return Integer.parseUnsignedInt(clean.substring(2), 16);
      }
      return Integer.parseInt(clean);
    } catch (NumberFormatException e) {
      return def;
    }
  }
}
