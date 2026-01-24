package frc.demacia.sysID;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.simple.SimpleMatrix;

public class LogReader {

    private static Map<Integer,List<EntryDescription>> entries;

    private static class EntryDescription {
        String name;
        String type;
        List<DataPoint> data = new ArrayList<>();

        EntryDescription(String name, String type) {
            this.name = name;
            this.type = type;
        }
    }

    private static class DataPoint {
        long timestamp;
        double[] value;

        DataPoint(long timestamp, double[] value) {
            this.timestamp = timestamp;
            this.value = value.clone();
        }
    }

    public static class SysIDResults {
        public BucketResult slow;
        public BucketResult mid;
        public BucketResult high;

        public SysIDResults(BucketResult slow, BucketResult mid, BucketResult high) {
            this.slow = slow;
            this.mid = mid;
            this.high = high;
        }
    }

    public static Map<String, SysIDResults> getResult(String fileName) {
        entries = new HashMap<>();
        try {
            System.out.println("Reading log: " + fileName);
            wpilogReader(fileName);
            return performAnalysis();
        } catch (IOException e) {
            System.err.println("Error: " + e.getMessage());
            e.printStackTrace();
            return new HashMap<>();
        }
    }

    private static void wpilogReader(String fileName) throws IOException {
        try (FileInputStream fileInputStream = new FileInputStream(fileName);
             DataInputStream dataInputStream = new DataInputStream(fileInputStream)) {
            
            byte[] signature = readHeader(dataInputStream);
            if (!Arrays.equals(signature, "WPILOG".getBytes())) {
                throw new IOException("Invalid WPILOG");
            }
            
            skipHeaderExtra(dataInputStream);
            readRecords(dataInputStream);
        }
    }

    private static byte[] readHeader(DataInputStream dataInputStream) throws IOException {
        byte[] signature = new byte[6];
        dataInputStream.readFully(signature);
        return signature;
    }

    private static void skipHeaderExtra(DataInputStream dataInputStream) throws IOException {
        dataInputStream.readShort();
        int extraLength = Integer.reverseBytes(dataInputStream.readInt());
        if(extraLength > 0) {
            dataInputStream.skipBytes(extraLength);
        }
    }

    private static void readRecords(DataInputStream dataInputStream) throws IOException {
        while (true) {
            try {
                readRecord(dataInputStream);
            } catch (EOFException e) {
                break;
            }
        }
    }

    private static boolean readRecord(DataInputStream dataInputStream) throws IOException {
        int headerByte = dataInputStream.readUnsignedByte();
        int idLength = (headerByte & 0x3) + 1;
        int payloadLength = (headerByte >> 2 & 0x3) + 1;
        int timestampLength = (headerByte >> 4 & 0x7) + 1;

        int recordId = readLittleEndianInt(dataInputStream, idLength);
        int payloadSize = readLittleEndianInt(dataInputStream, payloadLength);
        long timestamp = readLittleEndianLong(dataInputStream, timestampLength);

        if (recordId == 0) {
            addEntryFromControlRecord(dataInputStream, payloadSize);
            return false;
        } else {
            List<EntryDescription> entryList = entries.get(recordId);
            if (entryList != null && !entryList.isEmpty()) {
                String type = entryList.get(0).type.trim();
                boolean isFloat = type.equals("float") || type.equals("float[]");
                boolean isDouble = type.equals("double") || type.equals("double[]");

                if (isFloat || isDouble) {
                    double[] value = null;

                    if (isFloat) {
                        if (payloadSize % 4 == 0) {
                            int count = payloadSize / 4;
                            value = new double[count];
                            for (int i = 0; i < count; i++) {
                                int raw = Integer.reverseBytes(dataInputStream.readInt());
                                value[i] = (double) Float.intBitsToFloat(raw);
                            }
                        } else {
                            dataInputStream.skipBytes(payloadSize);
                        }
                    } else if (isDouble) {
                         if (payloadSize % 8 == 0) {
                            int count = payloadSize / 8;
                            value = new double[count];
                            for (int i = 0; i < count; i++) {
                                long raw = Long.reverseBytes(dataInputStream.readLong());
                                value[i] = Double.longBitsToDouble(raw);
                            }
                        } else {
                            dataInputStream.skipBytes(payloadSize);
                        }
                    }

                    if (value != null) {
                        int numEntries = entryList.size();
                        if (numEntries > 1 && value.length % numEntries == 0) {
                            int chunkSize = value.length / numEntries;
                            for (int i = 0; i < numEntries; i++) {
                                double[] slice = Arrays.copyOfRange(value, i * chunkSize, (i + 1) * chunkSize);
                                entryList.get(i).data.add(new DataPoint(timestamp, slice));
                            }
                        } else {
                            for (EntryDescription entry : entryList) {
                                entry.data.add(new DataPoint(timestamp, value));
                            }
                        }
                        return true;
                    }
                } else {
                    dataInputStream.skipBytes(payloadSize);
                }
            } else {
                dataInputStream.skipBytes(payloadSize);
            }
        }
        return false;
    }

    private static void addEntryFromControlRecord(DataInputStream dataInputStream, int payloadSize) throws IOException {
        int recordType = dataInputStream.readUnsignedByte();
        if (recordType == 0) { 
            int entryId = Integer.reverseBytes(dataInputStream.readInt());
            int nameLength = Integer.reverseBytes(dataInputStream.readInt());
            String name = readString(dataInputStream, nameLength);
            int typeLength = Integer.reverseBytes(dataInputStream.readInt());
            String type = readString(dataInputStream, typeLength);
            int metaLength = Integer.reverseBytes(dataInputStream.readInt());
            String metadata = readString(dataInputStream, metaLength);

            String[] names = name.split(" \\| ");
            String[] metas = metadata.split(" \\| ");
            
            for (int i = 0; i < names.length; i++) {
                String currentName = names[i].trim().split("\\: ")[0];
                String currentMeta = (i < metas.length) ? metas[i].trim() : "";

                if (currentMeta.contains("motor")) {
                    entries.putIfAbsent(entryId, new ArrayList<>());
                    entries.get(entryId).add(new EntryDescription(currentName, type));
                }
            }
        } else {
            if (payloadSize > 1) {
                dataInputStream.skipBytes(payloadSize - 1);
            }
        }
    }

    private static String readString(DataInputStream dataInputStream, int length) throws IOException {
        byte[] bytes = new byte[length];
        dataInputStream.readFully(bytes);
        return new String(bytes, "UTF-8");
    }

    private static int readLittleEndianInt(DataInputStream dis, int bytes) throws IOException {
        int result = 0;
        for (int i = 0; i < bytes; i++) {
            result |= (dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    private static long readLittleEndianLong(DataInputStream dis, int bytes) throws IOException {
        long result = 0L;
        for (int i = 0; i < bytes; i++) {
            result |= ((long) dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    private static Map<String, SysIDResults> performAnalysis() {
        Map<String, SysIDResults> results = new HashMap<>();
        Set<String> groups = findGroups();
        
        for (String group : groups) {
            SysIDResults result = analyzeGroup(group);
            if (result != null) {
                results.put(group, result);
            }
        }
        return results;
    }

    private static Set<String> findGroups() {
        Set<String> groups = new HashSet<>();
        for (List<EntryDescription> list : entries.values()) {
            for (EntryDescription entry : list) {
                groups.add(entry.name);
            }
        }
        return groups;
    }

    private static SysIDResults analyzeGroup(String name) {
        List<DataPoint> allData = new ArrayList<>();

        for (List<EntryDescription> list : entries.values()) {
            for (EntryDescription entry : list) {
                if (entry.name.equals(name)) {
                    allData.addAll(entry.data);
                }
            }
        }

        if (allData.isEmpty()) {
            return null;
        }

        allData.sort((p1, p2) -> Long.compare(p1.timestamp, p2.timestamp));

        List<SyncedDataPoint> syncedData = synchronizeData(allData);
        return performSysIdLikeAnalysis(syncedData);
    }

    public static class SyncedDataPoint {
        double velocity, position, acceleration, rawAcceleration, voltage;
        long timestamp;
        SyncedDataPoint prev;

        SyncedDataPoint(double velocity, double position, double acceleration, double voltage, long timestamp) {
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
            this.voltage = voltage;
            this.timestamp = timestamp;
        }
    }

    private static List<SyncedDataPoint> synchronizeData(List<DataPoint> dataPoints) {
        List<SyncedDataPoint> result = new ArrayList<>();
        for (DataPoint dp : dataPoints) {
            if (dp.value.length >= 4) {
                result.add(new SyncedDataPoint(dp.value[1], dp.value[0], dp.value[2], dp.value[3], dp.timestamp));
            }
        }
        updateAcceleration(result);
        return result;
    }

    private static void updateAcceleration(List<SyncedDataPoint> data) {
        SyncedDataPoint prev = null;
        for (SyncedDataPoint m : data) {
            m.rawAcceleration = m.acceleration;
            if (prev != null) {
                double deltaTime = (m.timestamp - prev.timestamp) / 1000000.0;
                if (deltaTime <= 0) deltaTime = 1e-6;
                
                double acc = (m.velocity - prev.velocity) / deltaTime;
                m.acceleration = (m.acceleration * deltaTime + acc * 0.02) / (deltaTime + 0.02);
            }
            m.prev = prev;
            prev = m;
        }
    }

    public static class BucketResult {
        double ks, kv, kv2, ka, kg, ksin, avgError, maxError;
        int points;

        BucketResult(double ks, double kv, double kv2, double ka, double kg, double ksin, double avgError, double maxError, int points) {
            this.ks = ks;
            this.kv = kv;
            this.kv2 = kv2;
            this.ka = ka;
            this.kg = kg;
            this.ksin = ksin;
            this.avgError = avgError;
            this.maxError = maxError;
            this.points = points;
        }
    }

    private static SysIDResults performSysIdLikeAnalysis(List<SyncedDataPoint> data) {
        if (data.isEmpty()) return new SysIDResults(null, null, null);

        double maxV = 0.0;
        for (SyncedDataPoint m : data) {
            maxV = Math.max(maxV, Math.abs(m.velocity));
        }

        double[] vRange = new double[]{maxV * 0.3, maxV * 0.7, maxV};
        List<SyncedDataPoint> slow = new ArrayList<>();
        List<SyncedDataPoint> mid = new ArrayList<>();
        List<SyncedDataPoint> high = new ArrayList<>();

        for (SyncedDataPoint d : data) {
            int r = rangeBucket(d, vRange);
            if (r == 0) slow.add(d);
            else if (r == 1) mid.add(d);
            else if (r == 2) high.add(d);
        }

        BucketResult slowR = solveBucket(slow);
        BucketResult midR = solveBucket(mid);
        BucketResult highR = solveBucket(high);

        return new SysIDResults(slowR, midR, highR);
    }

    private static int rangeBucket(SyncedDataPoint d, double[] vRange) {
        double vAbs = Math.abs(d.velocity);
        int i = vAbs < vRange[0] ? 0 : (vAbs < vRange[1] ? 1 : 2);
        
        if (valid(vAbs, 0.1) && valid(d.voltage, 0.05)) {
            if (d.prev != null) {
                if (valid(Math.abs(d.prev.velocity), 0.1) && valid(Math.abs(d.prev.voltage), 0.2)) {
                    return i;
                } else {
                    return -1;
                }
            } else {
                return i;
            }
        } else {
            return -1;
        }
    }

    private static boolean valid(double val, double min) {
        return Math.abs(val) > min;
    }

    private static BucketResult solveBucket(List<SyncedDataPoint> arr) {
        if (arr == null || arr.size() <= 50) return null;
        
        int rows = arr.size();
        
        int cols = 0;
        int idxSign = -1, idxVel = -1, idxAcc = -1, idxGrav = -1, idxSin = -1, idxKv2 = -1;

        if (SysidApp.kFlags[0]) idxSign = cols++;
        if (SysidApp.kFlags[1]) idxVel = cols++;
        if (SysidApp.kFlags[2]) idxAcc = cols++;
        if (SysidApp.kFlags[3]) idxGrav = cols++;
        if (SysidApp.kFlags[4]) idxSin = cols++;
        if (SysidApp.kFlags[5]) idxKv2 = cols++;

        SimpleMatrix mat = new SimpleMatrix(rows, cols);
        SimpleMatrix volt = new SimpleMatrix(rows, 1);
        
        for (int r = 0; r < rows; r++) {
            SyncedDataPoint d = arr.get(r);
            
            if (idxSign != -1) mat.set(r, idxSign, Math.signum(d.velocity));
            if (idxVel != -1) mat.set(r, idxVel, d.velocity);
            if (idxAcc != -1) mat.set(r, idxAcc, d.acceleration);
            if (idxGrav != -1) mat.set(r, idxGrav, 1.0);
            if (idxSin != -1) mat.set(r, idxSin, Math.cos(d.position)); 
            if (idxKv2 != -1) mat.set(r, idxKv2, d.velocity * Math.abs(d.velocity));
            
            volt.set(r, 0, d.voltage);
        }
        
        try {
            SimpleMatrix res = mat.solve(volt);
            SimpleMatrix pred = mat.mult(res);
            SimpleMatrix error = volt.minus(pred);
            
            double cKs = (idxSign != -1) ? res.get(idxSign, 0) : 0;
            double cKv = (idxVel != -1) ? res.get(idxVel, 0) : 0;
            double cKa = (idxAcc != -1) ? res.get(idxAcc, 0) : 0;
            double cKg = (idxGrav != -1) ? res.get(idxGrav, 0) : 0;
            double cKsin = (idxSin != -1) ? res.get(idxSin, 0) : 0;
            double cKv2 = (idxKv2 != -1) ? res.get(idxKv2, 0) : 0;
            
            double maxError = 0;
            double sumError = 0;
            int validCount = 0;
            
            for (int i = 0; i < rows; i++) {
                SyncedDataPoint d = arr.get(i);
                if (Math.abs(d.voltage) > 0.001) {
                    double relError = Math.abs(error.get(i, 0) / d.voltage);
                    sumError += relError;
                    maxError = Math.max(maxError, relError);
                    validCount++;
                }
            }
            double avgError = validCount > 0 ? sumError / validCount : 0;
            
            return new BucketResult(cKs, cKv, cKv2, cKa, cKg, cKsin, avgError, maxError, rows);
        } catch (Exception e) {
            System.err.println("Error solving bucket: " + e.getMessage());
            return null;
        }
    }
}