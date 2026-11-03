package frc.lib.fluxfig;

import java.io.BufferedOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Shared zero-dependency binary WPILib DataLog ({@code .wpilog} v1.0) reader and writer used across
 * {@code FluxProcessor}, {@code Flux-SystemCore-2027}, and {@code Flux-RoboRIO-2027}.
 */
public final class FluxWpiLogCodec {

  private static final byte[] MAGIC = "WPILOG".getBytes(StandardCharsets.US_ASCII);
  private static final int CONTROL_START = 0;
  private static final int CONTROL_SET_METADATA = 2;

  private FluxWpiLogCodec() {}

  public record EntryDef(int id, String name, String type, String metadata) {}

  public record DoubleSample(long timestampUs, double value) {}

  public record LongSample(long timestampUs, long value) {}

  public record DoubleArraySample(long timestampUs, double[] values) {}

  public record BooleanSample(long timestampUs, boolean value) {}

  public record StringSample(long timestampUs, String value) {}

  /** Parsed in-memory representation of a binary {@code .wpilog} file. */
  public record ParsedLog(
      Path sourcePath,
      String extraHeader,
      Map<Integer, EntryDef> entriesById,
      Map<String, EntryDef> entriesByName,
      Map<String, List<DoubleSample>> doubleSeries,
      Map<String, List<LongSample>> intSeries,
      Map<String, List<DoubleArraySample>> doubleArraySeries,
      Map<String, List<BooleanSample>> booleanSeries,
      Map<String, List<StringSample>> stringSeries,
      long minTimestampUs,
      long maxTimestampUs) {
    public ParsedLog {
      entriesById = Collections.unmodifiableMap(entriesById);
      entriesByName = Collections.unmodifiableMap(entriesByName);
      doubleSeries = Collections.unmodifiableMap(doubleSeries);
      intSeries = Collections.unmodifiableMap(intSeries);
      doubleArraySeries = Collections.unmodifiableMap(doubleArraySeries);
      booleanSeries = Collections.unmodifiableMap(booleanSeries);
      stringSeries = Collections.unmodifiableMap(stringSeries);
    }

    public List<DoubleSample> getDoubleSeries(String name) {
      return doubleSeries.getOrDefault(name, Collections.emptyList());
    }

    public List<LongSample> getIntSeries(String name) {
      return intSeries.getOrDefault(name, Collections.emptyList());
    }

    public List<DoubleArraySample> getDoubleArraySeries(String name) {
      return doubleArraySeries.getOrDefault(name, Collections.emptyList());
    }

    public List<BooleanSample> getBooleanSeries(String name) {
      return booleanSeries.getOrDefault(name, Collections.emptyList());
    }

    public List<StringSample> getStringSeries(String name) {
      return stringSeries.getOrDefault(name, Collections.emptyList());
    }

    public Map<String, List<DoubleSample>> allDoubleSeries() {
      return doubleSeries;
    }

    public Map<String, List<LongSample>> allIntSeries() {
      return intSeries;
    }

    public Map<String, List<DoubleArraySample>> allDoubleArraySeries() {
      return doubleArraySeries;
    }

    public Map<String, List<BooleanSample>> allBooleanSeries() {
      return booleanSeries;
    }

    public Map<String, List<StringSample>> allStringSeries() {
      return stringSeries;
    }

    public double durationSeconds() {
      return maxTimestampUs > minTimestampUs
          ? (maxTimestampUs - minTimestampUs) / 1_000_000.0
          : 0.0;
    }
  }

  public static final class Writer implements Closeable {
    private final OutputStream out;
    private final Map<String, Integer> entryIds = new LinkedHashMap<>();
    private final ByteBuffer hdrBuf = ByteBuffer.allocate(17).order(ByteOrder.LITTLE_ENDIAN);
    private final ByteBuffer buf8 = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    private final ByteBuffer buf24 = ByteBuffer.allocate(24).order(ByteOrder.LITTLE_ENDIAN);
    private int nextEntryId = 1;

    public Writer(Path outputPath, String extraHeader) throws IOException {
      if (outputPath.getParent() != null) {
        Files.createDirectories(outputPath.getParent());
      }
      this.out = new BufferedOutputStream(Files.newOutputStream(outputPath), 65536);
      byte[] extraBytes =
          (extraHeader != null ? extraHeader : "FLUX-2027").getBytes(StandardCharsets.UTF_8);
      ByteBuffer buf = ByteBuffer.allocate(12 + extraBytes.length).order(ByteOrder.LITTLE_ENDIAN);
      buf.put(MAGIC);
      buf.putShort((short) 0x0100);
      buf.putInt(extraBytes.length);
      buf.put(extraBytes);
      out.write(buf.array());
    }

    public int startEntry(String name, String type, String metadata, long timestampUs)
        throws IOException {
      Integer existing = entryIds.get(name);
      if (existing != null) {
        return existing;
      }
      int id = nextEntryId++;
      entryIds.put(name, id);
      byte[] nameBytes = name.getBytes(StandardCharsets.UTF_8);
      byte[] typeBytes = type.getBytes(StandardCharsets.UTF_8);
      byte[] metaBytes = (metadata != null ? metadata : "").getBytes(StandardCharsets.UTF_8);
      int payloadLen = 1 + 4 + 4 + nameBytes.length + 4 + typeBytes.length + 4 + metaBytes.length;
      ByteBuffer payload = ByteBuffer.allocate(payloadLen).order(ByteOrder.LITTLE_ENDIAN);
      payload.put((byte) CONTROL_START);
      payload.putInt(id);
      payload.putInt(nameBytes.length);
      payload.put(nameBytes);
      payload.putInt(typeBytes.length);
      payload.put(typeBytes);
      payload.putInt(metaBytes.length);
      payload.put(metaBytes);
      writeRecord(0, timestampUs, payload.array());
      return id;
    }

    public void appendDouble(int entryId, long timestampUs, double value) throws IOException {
      buf8.clear();
      buf8.putDouble(value);
      writeRecord(entryId, timestampUs, buf8.array());
    }

    public void appendDoubleArray(int entryId, long timestampUs, double[] values)
        throws IOException {
      ByteBuffer buf = ByteBuffer.allocate(values.length * 8).order(ByteOrder.LITTLE_ENDIAN);
      for (double v : values) {
        buf.putDouble(v);
      }
      writeRecord(entryId, timestampUs, buf.array());
    }

    public void appendBoolean(int entryId, long timestampUs, boolean value) throws IOException {
      writeRecord(entryId, timestampUs, new byte[] {(byte) (value ? 1 : 0)});
    }

    public void appendString(int entryId, long timestampUs, String value) throws IOException {
      writeRecord(
          entryId, timestampUs, (value != null ? value : "").getBytes(StandardCharsets.UTF_8));
    }

    public void appendInt64(int entryId, long timestampUs, long value) throws IOException {
      buf8.clear();
      buf8.putLong(value);
      writeRecord(entryId, timestampUs, buf8.array());
    }

    /**
     * Registers WPILib binary {@code structschema} entries for {@code Pose2d}, {@code
     * Translation2d}, {@code Rotation2d}, and {@code Translation3d} so AdvantageScope 2D/3D Field
     * tabs decode {@code struct:Pose2d} and {@code struct:Translation3d[]} topics automatically.
     */
    public void registerStandardGeometrySchemas(long timestampUs) throws IOException {
      writeStructSchema("Translation2d", "double x;double y", timestampUs);
      writeStructSchema("Rotation2d", "double value", timestampUs);
      writeStructSchema("Pose2d", "Translation2d translation;Rotation2d rotation", timestampUs);
      writeStructSchema("Translation3d", "double x;double y;double z", timestampUs);
    }

    private void writeStructSchema(String structName, String schemaDef, long timestampUs)
        throws IOException {
      String topic = "/.schema/struct:" + structName;
      if (entryIds.containsKey(topic)) {
        return;
      }
      int id = startEntry(topic, "structschema", "", timestampUs);
      writeRecord(id, timestampUs, schemaDef.getBytes(StandardCharsets.UTF_8));
    }

    /** Appends a WPILib {@code struct:Pose2d} (24 bytes: {@code x}, {@code y}, {@code rotRad}). */
    public void appendPose2d(
        int entryId, long timestampUs, double xMeters, double yMeters, double headingRad)
        throws IOException {
      buf24.clear();
      buf24.putDouble(xMeters);
      buf24.putDouble(yMeters);
      buf24.putDouble(headingRad);
      writeRecord(entryId, timestampUs, buf24.array());
    }

    /**
     * Appends a WPILib {@code struct:Translation3d[]} array ({@code 24 * N} bytes: packed {@code x,
     * y, z} triplets in meters).
     */
    public void appendTranslation3dArray(int entryId, long timestampUs, double[] xyzTriplets)
        throws IOException {
      ByteBuffer buf = ByteBuffer.allocate(xyzTriplets.length * 8).order(ByteOrder.LITTLE_ENDIAN);
      for (double v : xyzTriplets) {
        buf.putDouble(v);
      }
      writeRecord(entryId, timestampUs, buf.array());
    }

    public void writeRecord(int entryId, long timestampUs, byte[] payload) throws IOException {
      int headerByte = 3 | (3 << 2) | (7 << 4);
      hdrBuf.clear();
      hdrBuf.put((byte) headerByte);
      hdrBuf.putInt(entryId);
      hdrBuf.putInt(payload.length);
      hdrBuf.putLong(Math.max(0L, timestampUs));
      out.write(hdrBuf.array());
      out.write(payload);
    }

    @Override
    public void close() throws IOException {
      out.flush();
      out.close();
    }
  }

  public static ParsedLog read(Path path) throws IOException {
    try (FileChannel channel = FileChannel.open(path, StandardOpenOption.READ)) {
      long fileSize = channel.size();
      if (fileSize < 12) {
        throw new IOException("Invalid .wpilog file (too small): " + path);
      }
      ByteBuffer buf = channel.map(FileChannel.MapMode.READ_ONLY, 0, fileSize);
      buf.order(ByteOrder.LITTLE_ENDIAN);
      return parseBuffer(path, buf);
    }
  }

  public static ParsedLog parseBuffer(Path sourcePath, ByteBuffer buf) throws IOException {
    buf.order(ByteOrder.LITTLE_ENDIAN);
    for (byte b : MAGIC) {
      if (buf.get() != b) {
        throw new IOException("Invalid .wpilog magic header in " + sourcePath);
      }
    }
    int version = Short.toUnsignedInt(buf.getShort());
    if (version < 0x0100) {
      throw new IOException("Unsupported .wpilog version: " + Integer.toHexString(version));
    }
    int extraLen = buf.getInt();
    if (extraLen < 0 || extraLen > buf.remaining()) {
      throw new IOException("Corrupt extra header length in " + sourcePath);
    }
    byte[] extraBytes = new byte[extraLen];
    buf.get(extraBytes);
    String extraHeader = new String(extraBytes, StandardCharsets.UTF_8);

    Map<Integer, EntryDef> entriesById = new LinkedHashMap<>();
    Map<String, EntryDef> entriesByName = new LinkedHashMap<>();
    Map<String, List<DoubleSample>> doubleSeries = new LinkedHashMap<>();
    Map<String, List<LongSample>> intSeries = new LinkedHashMap<>();
    Map<String, List<DoubleArraySample>> doubleArraySeries = new LinkedHashMap<>();
    Map<String, List<BooleanSample>> booleanSeries = new LinkedHashMap<>();
    Map<String, List<StringSample>> stringSeries = new LinkedHashMap<>();
    long minTs = Long.MAX_VALUE;
    long maxTs = Long.MIN_VALUE;

    while (buf.hasRemaining()) {
      int hdr = Byte.toUnsignedInt(buf.get());
      int idLen = (hdr & 0x03) + 1;
      int sizeLen = ((hdr >> 2) & 0x03) + 1;
      int tsLen = ((hdr >> 4) & 0x07) + 1;
      if (buf.remaining() < idLen + sizeLen + tsLen) {
        break;
      }
      int entryId = (int) readUnsignedLe(buf, idLen);
      int payloadSize = (int) readUnsignedLe(buf, sizeLen);
      long ts = readUnsignedLe(buf, tsLen);
      if (payloadSize < 0 || payloadSize > buf.remaining()) {
        break;
      }
      int nextPos = buf.position() + payloadSize;
      if (entryId == 0 && payloadSize >= 5) {
        int ctrl = Byte.toUnsignedInt(buf.get());
        if (ctrl == CONTROL_START && buf.remaining() >= 12) {
          int startedId = buf.getInt();
          String name = readStr(buf, nextPos);
          String type = readStr(buf, nextPos);
          String metadata = readStr(buf, nextPos);
          if (name != null && type != null) {
            EntryDef def = new EntryDef(startedId, name, type, metadata != null ? metadata : "");
            entriesById.put(startedId, def);
            entriesByName.put(name, def);
          }
        } else if (ctrl == CONTROL_SET_METADATA && buf.remaining() >= 8) {
          int targetId = buf.getInt();
          String metadata = readStr(buf, nextPos);
          EntryDef old = entriesById.get(targetId);
          if (old != null && metadata != null) {
            EntryDef updated = new EntryDef(old.id(), old.name(), old.type(), metadata);
            entriesById.put(targetId, updated);
            entriesByName.put(old.name(), updated);
          }
        }
      } else {
        boolean isMonotonicTs = ts >= 0 && ts < 100_000_000_000L;
        if (isMonotonicTs) {
          if (ts > 0 && ts < minTs) {
            minTs = ts;
          }
          if (ts > maxTs) {
            maxTs = ts;
          }
        }
        EntryDef def = entriesById.get(entryId);
        if (def != null && isMonotonicTs) {
          String name = def.name();
          String type = def.type();
          if ("double".equals(type) && payloadSize == 8) {
            doubleSeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new DoubleSample(ts, buf.getDouble()));
          } else if ("float".equals(type) && payloadSize == 4) {
            doubleSeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new DoubleSample(ts, buf.getFloat()));
          } else if ("int64".equals(type) && payloadSize == 8) {
            long val = buf.getLong();
            intSeries.computeIfAbsent(name, k -> new ArrayList<>()).add(new LongSample(ts, val));
            doubleSeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new DoubleSample(ts, (double) val));
          } else if ("double[]".equals(type) && payloadSize >= 8 && (payloadSize % 8) == 0) {
            int count = payloadSize / 8;
            double[] arr = new double[count];
            for (int i = 0; i < count; i++) {
              double v = buf.getDouble();
              arr[i] = v;
              doubleSeries
                  .computeIfAbsent(name + "[" + i + "]", k -> new ArrayList<>())
                  .add(new DoubleSample(ts, v));
            }
            doubleArraySeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new DoubleArraySample(ts, arr));
          } else if ("float[]".equals(type) && payloadSize >= 4 && (payloadSize % 4) == 0) {
            int count = payloadSize / 4;
            for (int i = 0; i < count; i++) {
              double v = buf.getFloat();
              doubleSeries
                  .computeIfAbsent(name + "[" + i + "]", k -> new ArrayList<>())
                  .add(new DoubleSample(ts, v));
            }
          } else if ("int64[]".equals(type) && payloadSize >= 8 && (payloadSize % 8) == 0) {
            int count = payloadSize / 8;
            for (int i = 0; i < count; i++) {
              double v = buf.getLong();
              doubleSeries
                  .computeIfAbsent(name + "[" + i + "]", k -> new ArrayList<>())
                  .add(new DoubleSample(ts, v));
            }
          } else if ("boolean".equals(type) && payloadSize == 1) {
            booleanSeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new BooleanSample(ts, buf.get() != 0));
          } else if ("boolean[]".equals(type) && payloadSize >= 1) {
            for (int i = 0; i < payloadSize; i++) {
              boolean v = buf.get() != 0;
              booleanSeries
                  .computeIfAbsent(name + "[" + i + "]", k -> new ArrayList<>())
                  .add(new BooleanSample(ts, v));
            }
          } else if (("string".equals(type)
                  || "json".equals(type)
                  || "structschema".equals(type))
              && payloadSize >= 0) {
            byte[] sb = new byte[payloadSize];
            buf.get(sb);
            stringSeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new StringSample(ts, new String(sb, StandardCharsets.UTF_8)));
          } else if ("struct:Pose2d".equals(type) && payloadSize == 24) {
            double px = buf.getDouble();
            double py = buf.getDouble();
            double rotRad = buf.getDouble();
            doubleSeries
                .computeIfAbsent(name + ".x", k -> new ArrayList<>())
                .add(new DoubleSample(ts, px));
            doubleSeries
                .computeIfAbsent(name + ".y", k -> new ArrayList<>())
                .add(new DoubleSample(ts, py));
            doubleSeries
                .computeIfAbsent(name + ".rotRad", k -> new ArrayList<>())
                .add(new DoubleSample(ts, rotRad));
          } else if ("struct:Translation3d[]".equals(type)
              && payloadSize >= 0
              && payloadSize % 24 == 0) {
            int numBalls = payloadSize / 24;
            int coordCount = payloadSize / 8;
            double[] coords = new double[coordCount];
            for (int i = 0; i < coordCount; i++) {
              coords[i] = buf.getDouble();
            }
            doubleSeries
                .computeIfAbsent(name + ".count", k -> new ArrayList<>())
                .add(new DoubleSample(ts, numBalls));
            doubleArraySeries
                .computeIfAbsent(name, k -> new ArrayList<>())
                .add(new DoubleArraySample(ts, coords));
          }
        }
      }
      buf.position(nextPos);
    }
    if (minTs == Long.MAX_VALUE) {
      minTs = 0;
      maxTs = 0;
    }
    return new ParsedLog(
        sourcePath,
        extraHeader,
        entriesById,
        entriesByName,
        doubleSeries,
        intSeries,
        doubleArraySeries,
        booleanSeries,
        stringSeries,
        minTs,
        maxTs);
  }

  private static long readUnsignedLe(ByteBuffer buf, int numBytes) {
    long res = 0;
    for (int i = 0; i < numBytes; i++) {
      res |= ((long) Byte.toUnsignedInt(buf.get())) << (8 * i);
    }
    return res;
  }

  private static String readStr(ByteBuffer buf, int limitPos) {
    if (buf.position() + 4 > limitPos) {
      return null;
    }
    int len = buf.getInt();
    if (len < 0 || buf.position() + len > limitPos) {
      return null;
    }
    byte[] bytes = new byte[len];
    buf.get(bytes);
    return new String(bytes, StandardCharsets.UTF_8);
  }
}
