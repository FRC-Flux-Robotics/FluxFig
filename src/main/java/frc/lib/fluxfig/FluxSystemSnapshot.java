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
   * Single non-intrusive shell command executed over SSH after a run to capture Linux {@code /proc}
   * and {@code /sys} counters on SystemCore or roboRIO without touching robot Java code.
   */
  public static final String REMOTE_CAPTURE_SH_COMMAND =
      "echo '[PLATFORM]'; "
          + "if [ -d /home/systemcore ] || ip link show can_s0 >/dev/null 2>&1; then echo 'controllerType=SYSTEMCORE'; else echo 'controllerType=ROBORIO'; fi; "
          + "echo \"hostname=$(hostname 2>/dev/null || echo unknown)\"; "
          + "echo \"kernelVersion=$(uname -r 2>/dev/null || echo unknown)\"; "
          + "awk '{print \"uptimeSec=\"$1}' /proc/uptime 2>/dev/null || true; "
          + "echo '[CPU_AND_IRQ]'; "
          + "awk '/^cpu / {total=$2+$3+$4+$5+$6+$7+$8; if (total>0) {printf \"totalCpuPct=%.1f\\nioWaitPct=%.1f\\nhardIrqPct=%.1f\\nsoftIrqPct=%.1f\\n\", (total-$5)*100.0/total, $6*100.0/total, $7*100.0/total, $8*100.0/total}}' /proc/stat 2>/dev/null || true; "
          + "ps -eo pcpu,comm 2>/dev/null | awk '/java/ {j+=$1} /limelight|photon/ {v+=$1} END {if (j>0) printf \"javaProcessCpuPct=%.1f\\n\", j; if (v>0) printf \"visionDaemonCpuPct=%.1f\\n\", v}' || true; "
          + "echo '[THERMAL_AND_POWER]'; "
          + "if [ -f /sys/class/thermal/thermal_zone0/temp ]; then awk '{printf \"cpuTempC=%.1f\\n\", $1/1000.0}' /sys/class/thermal/thermal_zone0/temp; fi; "
          + "if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]; then awk '{printf \"cpuFreqMHz=%.0f\\n\", $1/1000.0}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq; fi; "
          + "vcgencmd get_throttled 2>/dev/null | sed 's/throttled=/throttledHex=/' || true; "
          + "echo '[SOCKETCAN]'; "
          + "for d in /sys/class/net/can_*; do "
          + "  [ -e \"$d\" ] || continue; "
          + "  b=$(basename \"$d\"); "
          + "  rxp=$(cat \"$d/statistics/rx_packets\" 2>/dev/null || echo 0); "
          + "  txp=$(cat \"$d/statistics/tx_packets\" 2>/dev/null || echo 0); "
          + "  rxd=$(cat \"$d/statistics/rx_dropped\" 2>/dev/null || echo 0); "
          + "  txd=$(cat \"$d/statistics/tx_dropped\" 2>/dev/null || echo 0); "
          + "  rxo=$(cat \"$d/statistics/rx_over_errors\" 2>/dev/null || echo 0); "
          + "  st=$(ip -d link show \"$b\" 2>/dev/null | awk '/can state/ {for(i=1;i<=NF;i++) if($i==\"state\") print $(i+1)}'); "
          + "  echo \"can.${b}=${rxp},${txp},${rxd},${txd},${rxo},0,0,${st:-ERROR-ACTIVE}\"; "
          + "done; "
          + "echo '[STORAGE]'; "
          + "df -BM /U/logs /home/systemcore /home/lvuser 2>/dev/null | awk 'NR==2 {gsub(/M/,\"\",$4); print \"freeStorageMb=\"$4}' || true";

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

  /** Serializes this snapshot into the canonical {@code system-snapshot.txt} format. */
  public String serialize() {
    StringBuilder sb = new StringBuilder(1024);
    sb.append("# FLUX 10413 Offline System & Controller Snapshot\n");
    sb.append("[PLATFORM]\n");
    sb.append("controllerType=").append(controllerType).append('\n');
    sb.append("hostname=").append(hostname).append('\n');
    sb.append("kernelVersion=").append(kernelVersion).append('\n');
    sb.append(String.format(Locale.US, "uptimeSec=%.1f\n", uptimeSec));

    sb.append("\n[LOOP_AND_TIMING]\n");
    sb.append(String.format(Locale.US, "loopPeriodMs=%.2f\n", loopPeriodMs));
    sb.append(String.format(Locale.US, "loopMedianMs=%.2f\n", loopMedianMs));
    sb.append(String.format(Locale.US, "loopP99Ms=%.2f\n", loopP99Ms));
    sb.append(String.format(Locale.US, "loopMaxMs=%.2f\n", loopMaxMs));
    sb.append(String.format(Locale.US, "firstEnablePeakMs=%.2f\n", firstEnablePeakMs));
    sb.append(String.format(Locale.US, "odometryJitterMaxMs=%.2f\n", odometryJitterMaxMs));
    sb.append(String.format(Locale.US, "driveTraceMaxMs=%.2f\n", driveTraceMaxMs));
    sb.append(String.format(Locale.US, "visionTraceMaxMs=%.2f\n", visionTraceMaxMs));
    sb.append(
        String.format(Locale.US, "superstructureTraceMaxMs=%.2f\n", superstructureTraceMaxMs));

    sb.append("\n[CPU_AND_IRQ]\n");
    sb.append(String.format(Locale.US, "totalCpuPct=%.1f\n", totalCpuPct));
    sb.append(String.format(Locale.US, "javaProcessCpuPct=%.1f\n", javaProcessCpuPct));
    sb.append(String.format(Locale.US, "visionDaemonCpuPct=%.1f\n", visionDaemonCpuPct));
    sb.append(String.format(Locale.US, "softIrqPct=%.1f\n", softIrqPct));
    sb.append(String.format(Locale.US, "hardIrqPct=%.1f\n", hardIrqPct));
    sb.append(String.format(Locale.US, "ioWaitPct=%.1f\n", ioWaitPct));
    sb.append("spiCanIrqCount=").append(spiCanIrqCount).append('\n');
    sb.append("usbXhciIrqCount=").append(usbXhciIrqCount).append('\n');

    sb.append("\n[THERMAL_AND_POWER]\n");
    sb.append(String.format(Locale.US, "cpuTempC=%.1f\n", cpuTempC));
    sb.append(String.format(Locale.US, "cpuFreqMHz=%.0f\n", cpuFreqMHz));
    sb.append(String.format(Locale.US, "throttledHex=0x%x\n", throttledFlags));

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
    sb.append(String.format(Locale.US, "heapAllocRateMbPerSec=%.2f\n", heapAllocRateMbPerSec));
    sb.append("gcCount=").append(gcCount).append('\n');
    sb.append(String.format(Locale.US, "gcTotalPauseMs=%.2f\n", gcTotalPauseMs));
    sb.append("gcCorrelatedLoopSpikes=").append(gcCorrelatedLoopSpikes).append('\n');
    sb.append(String.format(Locale.US, "freeStorageMb=%.1f\n", freeStorageMb));
    sb.append(String.format(Locale.US, "logWriteLatencyMaxMs=%.2f\n", logWriteLatencyMaxMs));

    sb.append("\n[HARDWARE_PMU]\n");
    sb.append(String.format(Locale.US, "ipc=%.2f\n", ipc));
    sb.append(String.format(Locale.US, "l1dCacheMissPct=%.2f\n", l1dCacheMissPct));
    sb.append(String.format(Locale.US, "llcMissPct=%.2f\n", llcMissPct));
    sb.append("involuntaryContextSwitches=").append(involuntaryContextSwitches).append('\n');
    return sb.toString();
  }

  /** Parses {@code system-snapshot.txt} text into a structured {@link FluxSystemSnapshot}. */
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

    String controllerType = kv.getOrDefault("controllerType", "SYSTEMCORE");
    double defaultPeriod = "ROBORIO".equalsIgnoreCase(controllerType) ? 20.0 : 5.0;
    double defaultFreq = "ROBORIO".equalsIgnoreCase(controllerType) ? 667.0 : 2400.0;

    return new FluxSystemSnapshot(
        controllerType,
        kv.getOrDefault("hostname", "SystemCore-10413"),
        kv.getOrDefault("kernelVersion", "6.6.47-rt39"),
        parseDouble(kv.get("uptimeSec"), 0.0),
        parseDouble(kv.get("loopPeriodMs"), defaultPeriod),
        parseDouble(kv.get("loopMedianMs"), 1.5),
        parseDouble(kv.get("loopP99Ms"), 2.8),
        parseDouble(kv.get("loopMaxMs"), 4.1),
        parseDouble(kv.get("firstEnablePeakMs"), 3.9),
        parseDouble(kv.get("odometryJitterMaxMs"), 0.4),
        parseDouble(kv.get("driveTraceMaxMs"), 0.5),
        parseDouble(kv.get("visionTraceMaxMs"), 0.6),
        parseDouble(kv.get("superstructureTraceMaxMs"), 0.3),
        parseDouble(kv.get("totalCpuPct"), 40.0),
        parseDouble(kv.get("javaProcessCpuPct"), 12.0),
        parseDouble(kv.get("visionDaemonCpuPct"), 24.0),
        parseDouble(kv.get("softIrqPct"), 4.5),
        parseDouble(kv.get("hardIrqPct"), 1.2),
        parseDouble(kv.getOrDefault("ioWaitPct", kv.get("iowaitPct")), 0.4),
        parseLong(kv.get("spiCanIrqCount"), 0L),
        parseLong(kv.get("usbXhciIrqCount"), 0L),
        parseDouble(kv.get("cpuTempC"), 55.0),
        parseDouble(kv.get("cpuFreqMHz"), defaultFreq),
        parseHexOrInt(kv.get("throttledHex"), 0),
        parseDouble(kv.get("heapAllocRateMbPerSec"), 0.2),
        (int) parseLong(kv.get("gcCount"), 0L),
        parseDouble(kv.get("gcTotalPauseMs"), 0.0),
        (int) parseLong(kv.get("gcCorrelatedLoopSpikes"), 0L),
        parseDouble(kv.get("freeStorageMb"), 1024.0),
        parseDouble(kv.get("logWriteLatencyMaxMs"), 2.0),
        parseDouble(kv.get("ipc"), 1.4),
        parseDouble(kv.get("l1dCacheMissPct"), 1.8),
        parseDouble(kv.get("llcMissPct"), 3.2),
        parseLong(kv.get("involuntaryContextSwitches"), 10L),
        Collections.unmodifiableList(cans));
  }

  /** Evaluates all 14 offline watch metrics on this snapshot. */
  public List<WatchDecision> evaluate() {
    return evaluate(this, true, true);
  }

  /**
   * Evaluates all 14 offline watch metrics across 6 categories and maps every detected anomaly to a
   * concrete engineering, {@code FluxFig} CSV, or pit maintenance decision.
   *
   * @param s parsed or synthesized {@link FluxSystemSnapshot}
   * @param hasSnapshotFile whether a physical {@code system-snapshot.txt} file was present
   * @param evaluateStorage whether {@code freeStorageMb} was explicitly recorded in the snapshot
   */
  public static List<WatchDecision> evaluate(
      FluxSystemSnapshot s, boolean hasSnapshotFile, boolean evaluateStorage) {
    List<WatchDecision> out = new ArrayList<>();

    double p99LimitMs = s.loopPeriodMs() * 1.30; // 6.5 ms @ 200 Hz, 26.0 ms @ 50 Hz
    if (s.loopP99Ms() > p99LimitMs) {
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "Loop Period Jitter (p99 Δt)",
              String.format(
                  Locale.US, "p99=%.2f ms (max=%.2f ms)", s.loopP99Ms(), s.loopMaxMs()),
              String.format(Locale.US, "<= %.2f ms", p99LimitMs),
              s.loopP99Ms() > s.loopPeriodMs() * 1.75 ? "CRITICAL" : "WARN",
              String.format(
                  Locale.US,
                  "Control loop p99 (%.2f ms) exceeded %.0f Hz budget (%.1f ms period).",
                  s.loopP99Ms(),
                  1000.0 / s.loopPeriodMs(),
                  s.loopPeriodMs()),
              "Decision: Change loopPeriodMs in parameters.csv (FluxFig) from 5 ms (200 Hz) to 10 ms (100 Hz) so ChassisSpeeds.discretize() and PID dt stay deterministic."));
    }

    if (s.visionTraceMaxMs() > 2.5
        || s.driveTraceMaxMs() > 2.0
        || s.superstructureTraceMaxMs() > 1.5) {
      String culprit =
          s.visionTraceMaxMs() >= s.driveTraceMaxMs()
                  && s.visionTraceMaxMs() >= s.superstructureTraceMaxMs()
              ? "VisionMs"
              : (s.driveTraceMaxMs() >= s.superstructureTraceMaxMs()
                  ? "DriveMs"
                  : "SuperstructureMs");
      double peak =
          Math.max(
              s.visionTraceMaxMs(), Math.max(s.driveTraceMaxMs(), s.superstructureTraceMaxMs()));
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "Per-Subsystem Execution Breakdown (" + culprit + ")",
              String.format(
                  Locale.US,
                  "Drive=%.2f ms, Vision=%.2f ms, Super=%.2f ms",
                  s.driveTraceMaxMs(),
                  s.visionTraceMaxMs(),
                  s.superstructureTraceMaxMs()),
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

    if (s.firstEnablePeakMs() > Math.max(12.0, s.loopPeriodMs() * 2.4)
        && s.loopMedianMs() > 0.0
        && s.loopMedianMs() < s.loopPeriodMs() * 0.7) {
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

    if (s.odometryJitterMaxMs() > 1.5) {
      out.add(
          new WatchDecision(
              "1. Loop & Thread Timing",
              "250 Hz Odometry Thread Jitter",
              String.format(Locale.US, "±%.2f ms around 4.0 ms", s.odometryJitterMaxMs()),
              "<= ±1.50 ms",
              s.odometryJitterMaxMs() > 2.5 ? "CRITICAL" : "WARN",
              "250 Hz PhoenixOdometryThread experienced scheduling jitter.",
              "Decision: Pin PhoenixOdometryThread to Core 3 (taskset -c 3) away from Linux USB/Network interrupt handlers on Core 0."));
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
              && !"UNKNOWN".equalsIgnoreCase(can.canState());
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

    if (s.softIrqPct() > 25.0) {
      out.add(
          new WatchDecision(
              "2. SocketCAN & SPI Bus",
              "Kernel softirq & SPI Interrupt Load",
              String.format(
                  Locale.US, "softirq=%.1f%%, hardirq=%.1f%%", s.softIrqPct(), s.hardIrqPct()),
              "softirq <= 25.0%",
              s.softIrqPct() > 35.0 ? "CRITICAL" : "WARN",
              "Linux kernel softirq processing (SPI SocketCAN + USB UVC cameras) is saturating CPU0.",
              "Decision: Balance CAN status frame rates or lower OV9281 USB camera capture rate from 90 FPS -> 60 FPS."));
    }

    if (s.totalCpuPct() > 80.0 && s.javaProcessCpuPct() < 30.0) {
      out.add(
          new WatchDecision(
              "3. CPU, Vision & Thermal",
              "Java CPU % vs. Native Vision Daemon CPU %",
              String.format(
                  Locale.US,
                  "Total=%.1f%% (Java=%.1f%%, Vision C++=%.1f%%)",
                  s.totalCpuPct(),
                  s.javaProcessCpuPct(),
                  s.visionDaemonCpuPct()),
              "Total CPU <= 80.0%",
              "WARN",
              "Native C++ limelightsc0..3 AprilTag daemons (not Java robot code) are saturating the Cortex-A76 cores.",
              "Decision: Enable 2x detector decimation during Teleop or reduce camera FPS in cameras.csv."));
    }

    boolean thermalThrottled = (s.throttledFlags() & 0x40004) != 0;
    boolean freqThrottled =
        hasSnapshotFile
            && "SYSTEMCORE".equalsIgnoreCase(s.controllerType())
            && s.cpuFreqMHz() > 0.0
            && s.cpuFreqMHz() < 2000.0;
    if (s.cpuTempC() >= 80.0 || thermalThrottled || freqThrottled) {
      out.add(
          new WatchDecision(
              "3. CPU, Vision & Thermal",
              "CPU Die Temperature & CM5 Thermal Throttling",
              String.format(
                  Locale.US,
                  "%.1f °C @ %.0f MHz (throttled=0x%x)",
                  s.cpuTempC(),
                  s.cpuFreqMHz(),
                  s.throttledFlags()),
              "< 80.0 °C, 2400 MHz, 0x0",
              s.cpuTempC() >= 85.0 || (s.throttledFlags() & 0x4) != 0 ? "CRITICAL" : "WARN",
              "CM5 SoC reached thermal throttle threshold inside electronics enclosure.",
              "Decision: Add a 5V pit cooling fan over SystemCore heatsink or throttle idle camera processing while Disabled after PreMatchCalibrator locks."));
    }

    boolean underVoltage = (s.throttledFlags() & 0x10001) != 0;
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

    if (s.heapAllocRateMbPerSec() > 2.0) {
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
      out.add(
          new WatchDecision(
              "4. JVM Memory & GC",
              "GC Pause <-> Loop Overrun Correlation",
              String.format(
                  Locale.US,
                  "%d loop overruns aligned with GC (%d GCs, %.1f ms total)",
                  s.gcCorrelatedLoopSpikes(),
                  s.gcCount(),
                  s.gcTotalPauseMs()),
              "0 GC-induced loop overruns",
              s.gcCorrelatedLoopSpikes() >= 3 ? "CRITICAL" : "WARN",
              "JVM Garbage Collection pauses directly caused control loop overruns.",
              "Decision: Update JVM launch flags in build.gradle to -XX:+UseZGC -XX:+AlwaysPreTouch -Xms512m -Xmx512m."));
    }

    if (hasSnapshotFile
        && ((evaluateStorage && s.freeStorageMb() < 400.0)
            || s.ioWaitPct() > 5.0
            || s.logWriteLatencyMaxMs() > 25.0)) {
      out.add(
          new WatchDecision(
              "5. Storage & eMMC I/O",
              "eMMC Free Space & Log I/O Wait",
              String.format(
                  Locale.US,
                  "Free=%.0f MB, iowait=%.1f%%, writeMax=%.1f ms",
                  s.freeStorageMb(),
                  s.ioWaitPct(),
                  s.logWriteLatencyMaxMs()),
              "Free >= 400 MB, iowait <= 5.0%",
              (evaluateStorage && s.freeStorageMb() < 150.0) ? "CRITICAL" : "WARN",
              "eMMC storage pressure or disk write latency spikes detected during match logging.",
              "Decision: Switch camera recording from continuous 15 FPS (~48 MB/match) to the Hybrid Event-Driven preset (~4.5 MB/match) and clean old logs."));
    }

    if (hasSnapshotFile
        && ((s.ipc() > 0.0 && s.ipc() < 0.80)
            || s.l1dCacheMissPct() > 8.0
            || s.llcMissPct() > 15.0
            || s.involuntaryContextSwitches() > 500L)) {
      out.add(
          new WatchDecision(
              "6. ARMv8 Hardware PMU",
              "Cortex-A76 IPC, L3 Cache Misses & Context Switches",
              String.format(
                  Locale.US,
                  "IPC=%.2f, L1D Miss=%.1f%%, LLC Miss=%.1f%%, InvolCtxSw=%d",
                  s.ipc(),
                  s.l1dCacheMissPct(),
                  s.llcMissPct(),
                  s.involuntaryContextSwitches()),
              "IPC >= 0.80, L1D <= 8.0%, LLC Miss <= 15.0%, InvolCtxSw <= 500",
              "WARN",
              "4x OV9281 camera pipelines are thrashing the shared 2 MB L3 cache or preempting real-time threads.",
              "Decision: Isolate Core 3 (isolcpus=3 / taskset) for the Java robot process + 250 Hz odometry thread and pin limelightsc0..3 daemons to Cores 0-2."));
    }

    return Collections.unmodifiableList(out);
  }

  private static double parseDouble(String s, double def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static long parseLong(String s, long def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Long.parseLong(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static int parseHexOrInt(String s, int def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    String clean = s.trim().toLowerCase(Locale.ROOT);
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
