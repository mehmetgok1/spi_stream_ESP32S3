import numpy as np
import cv2
import os
import csv

# 1. KESİN STRUCT TANIMI (sizeof = 24633 bytes)
combined_packet_dtype = np.dtype([
    ('batteryLevel', 'f4'),
    ('batteryPercentage', 'f4'),
    ('ambLight', 'f4'),
    ('ambLight_Int', 'u2'),
    ('PIRValue', 'f4'),
    ('movingDist', 'u2'),
    ('movingEnergy', 'u1'),
    ('staticDist', 'u2'),
    ('staticEnergy', 'u1'),
    ('detectionDist', 'u2'),
    # SlaveDataPacket başlangıcı
    ('sequence', 'u2'),
    ('ambientLight_slave', 'u2'),
    ('temperature', 'f4'),
    ('humidity', 'f4'),
    ('accelX', 'i2'), ('accelY', 'i2'), ('accelZ', 'i2'),
    ('gyroX', 'i2'), ('gyroY', 'i2'), ('gyroZ', 'i2'),
    ('timestamp_ms', 'u4'),
    ('status', 'u1'),
    ('accelSampleCount', 'u2'),
    ('accelX_samples', 'i2', (2000,)), 
    ('accelY_samples', 'i2', (2000,)),
    ('accelZ_samples', 'i2', (2000,)),
    ('microphoneSamples', 'u2', (2000,)),
    ('rgbFrame', 'u2', (4096,)),  # 64x64
    ('irFrame', 'u2', (192,))     # 16x12
])

# AYARLAR
session_path = "/media/deso/BOOT/20260330_161712"
session_id = os.path.basename(session_path)
output_base = os.path.join("./processed_sessions", session_id)

# Alt klasörleri oluştur
dirs = {
    "rgb": os.path.join(output_base, "colored_image"),
    "ir": os.path.join(output_base, "irimage"),
    "motion_audio": os.path.join(output_base, "accel_mic"),
    "sensors": os.path.join(output_base, "sensordata")
}
for d in dirs.values(): os.makedirs(d, exist_ok=True)

# Ana CSV dosyalarını hazırla
sensor_csv_path = os.path.join(dirs["sensors"], "all_sensors.csv")
motion_csv_path = os.path.join(dirs["motion_audio"], "accel_mic_stream.csv")

print(f"Oturum işleniyor: {session_id}")

# 2. TÜM BIN DOSYALARINI SIRALA VE OKU
bin_files = sorted([f for f in os.listdir(session_path) if f.endswith(".bin")])

with open(sensor_csv_path, 'w', newline='') as f_sensor, \
     open(motion_csv_path, 'w', newline='') as f_motion:
    
    writer_s = csv.writer(f_sensor)
    writer_m = csv.writer(f_motion)

    # Başlıkları yaz
    writer_s.writerow(["timestamp_ms", "sequence", "battery_pct", "ambLight_M", "PIR", "mmWave_dist", "temp", "humi", "ambLight_S"])
    writer_m.writerow(["timestamp_ms", "accelX", "accelY", "accelZ", "mic"])

    for bin_file in bin_files:
        full_path = os.path.join(session_path, bin_file)
        data = np.fromfile(full_path, dtype=combined_packet_dtype)
        print(f"Dosya okundu: {bin_file} ({len(data)} paket)")

        for packet in data:
            ts = packet['timestamp_ms']
            seq = packet['sequence']

            # --- A. SENSOR DATA (CSV) ---
            writer_s.writerow([ts, seq, packet['batteryPercentage'], packet['ambLight'], 
                               packet['PIRValue'], packet['movingDist'], packet['temperature'], 
                               packet['humidity'], packet['ambientLight_slave']])

            # --- B. ACCEL & MIC (CSV) ---
            # 2000 satır boyunca her örneği yaz
            for j in range(2000):
                writer_m.writerow([ts, packet['accelX_samples'][j], packet['accelY_samples'][j], 
                                   packet['accelZ_samples'][j], packet['microphoneSamples'][j]])

            # --- C. RGB IMAGE (PNG) ---
            rgb_raw = packet['rgbFrame'].view(np.uint8).reshape((64, 64, 2))
            img_bgr = cv2.cvtColor(rgb_raw, cv2.COLOR_BGR5652BGR)
            large_rgb = cv2.resize(img_bgr, (256, 256), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(os.path.join(dirs["rgb"], f"rgb_{ts}_{seq}.png"), large_rgb)

            # --- D. IR IMAGE (CSV) ---
            # Her paket için ayrı 16x12 csv
            ir_path = os.path.join(dirs["ir"], f"ir_{ts}_{seq}.csv")
            ir_matrix = packet['irFrame'].reshape((12, 16))
            np.savetxt(ir_path, ir_matrix, delimiter=",", fmt='%u')

print(f"\nİşlem Başarıyla Tamamlandı!")
print(f"Çıktı klasörü: {output_base}")
