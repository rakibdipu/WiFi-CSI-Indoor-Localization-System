# Wi-Fi CSI Device-Free Indoor Human Localization System

> **ESP32-S3 Channel State Information (CSI) Sensing, Machine Learning, and Real-Time 2D Digital Twin**

---

## 📁 Project Structure

```
capstone 3.2/
│
├── COMPLETE_PROJECT_DOCUMENTATION.md  <-- Complete Master Scientific & Engineering Guide
│
├── firmware/
│   └── esp32-csi-node/                <-- ESP-IDF v5.x Firmware for ESP32-S3-CAM (N16R8)
│       ├── CMakeLists.txt
│       ├── sdkconfig.defaults         <-- Optimized for 16MB Flash, 8MB Octal PSRAM, 921600 Baud
│       └── main/
│           ├── CMakeLists.txt
│           ├── config.h               <-- Set Phone Hotspot SSID & Password here
│           ├── wifi_csi.h             <-- CSI packet struct & API
│           ├── wifi_csi.c             <-- High-priority ISR callback + Core 1 CSV formatter
│           └── main.c                 <-- Wi-Fi STA connection + UDP traffic generator
│
├── tools/
│   └── csi_receiver.py                <-- High-speed serial receiver, dataset logger & stats
│
├── preprocessing/
│   └── csi_pipeline.py                <-- Hampel filter, Butterworth bandpass, Phase detrending, PCA
│
├── features/
│   └── extract_features.py            <-- Time/Frequency/Doppler/Spectral energy features
│
├── models/
│   ├── train_presence_and_localization.py <-- Baseline comparison, RF/KNN/Regressor training
│   ├── presence_model.pkl             <-- Serialized Presence Model
│   ├── feature_scaler.pkl             <-- Fitted StandardScaler
│   └── localization_regressor.pkl     <-- Serialized (X,Y) Regressor
│
├── backend/
│   └── server.py                      <-- FastAPI + WebSocket server + 2D Kalman Filter
│
├── dashboard/
│   ├── index.html                     <-- 2D Room Canvas & Real-time Digital Twin
│   └── app.js                         <-- Canvas rendering & Chart.js CSI amplitude spectrum
│
└── data/
    ├── raw/                           <-- Recorded CSV datasets
    └── processed/                     <-- Cleaned feature matrices
```

---

## 🚀 Quick Start Guide (Windows)

### 1. Configure & Flash Firmware to ESP32-S3
1. Open [`firmware/esp32-csi-node/main/config.h`](file:///C:/Users/ASUS/Downloads/capstone%203.2/firmware/esp32-csi-node/main/config.h).
2. Set your phone hotspot credentials:
   ```c
   #define WIFI_SSID       "YourPhoneHotspotName"
   #define WIFI_PASSWORD   "YourHotspotPassword"
   ```
3. Open ESP-IDF 5.x Command Prompt / PowerShell:
   ```powershell
   cd "firmware\esp32-csi-node"
   idf.py set-target esp32s3
   idf.py build
   idf.py -p COM3 -b 921600 flash monitor
   ```

### 2. Collect Data
```powershell
# Collect 60 seconds of Empty Room baseline
python tools/csi_receiver.py --port COM3 --duration 60 --label-presence 0 --label-activity "empty" --output data/raw/empty.csv

# Collect 60 seconds standing in Zone A1 (1.0m, 1.0m)
python tools/csi_receiver.py --port COM3 --duration 60 --label-presence 1 --label-activity "standing" --label-zone "A1" --label-x 1.0 --label-y 1.0 --output data/raw/zone_A1.csv
```

### 3. Train & Evaluate Models
```powershell
python models/train_presence_and_localization.py
```

### 4. Launch Live Digital Twin Dashboard
```powershell
python backend/server.py
```
Open **`http://localhost:8000`** in your browser.
