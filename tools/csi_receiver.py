#!/usr/bin/env python3
"""
tools/csi_receiver.py — High-Speed Serial CSI Packet Receiver & Dataset Logger

Connects to ESP32-S3 via USB-UART, parses incoming CSI_DATA frames,
calculates subcarrier amplitudes and phases, displays live transmission stats,
and logs structured CSI frames to CSV files for dataset collection and training.

Usage:
    python tools/csi_receiver.py --port COM3 --baud 921600 --output data/raw/dataset_sample.csv --label-zone "A1" --label-x 1.0 --label-y 1.0 --label-activity "standing"
"""

import sys
import os
import time
import argparse
import serial
import numpy as np
import pandas as pd
from datetime import datetime

# Subcarrier mapping for 20MHz HT-LTF on ESP32-S3 (ESP-IDF subcarrier index table)
# Hardware outputs indices in non-linear order [0..31, -32..-1]. We map them monotonically from -28 to +28.
SUBCARRIER_MAP_20MHZ_HT = [
    # Positive subcarriers (1 to 28) and Negative subcarriers (-28 to -1) excluding null/pilot carriers
    i for i in range(-28, 29) if i != 0
]

def parse_csi_line(line: str):
    """
    Parses a single raw line emitted by ESP32:
    Format: CSI_DATA,<timestamp_us>,<rssi>,<noise_floor>,<channel>,<sec_ch>,<bandwidth>,<sig_mode>,<antenna>,<num_pairs>,I0,Q0,I1,Q1,...
    """
    line = line.strip()
    if not line.startswith("CSI_DATA,"):
        return None
    
    parts = line.split(",")
    if len(parts) < 10:
        return None
    
    try:
        timestamp_us = int(parts[1])
        rssi = int(parts[2])
        noise_floor = int(parts[3])
        channel = int(parts[4])
        sec_ch = int(parts[5])
        bandwidth = int(parts[6])
        sig_mode = int(parts[7])
        antenna = int(parts[8])
        num_pairs = int(parts[9])
        
        iq_raw = parts[10:]
        if len(iq_raw) < num_pairs * 2:
            return None
        
        # Parse Real (I) and Imaginary (Q) components
        iq_values = np.array([int(v) for v in iq_raw[:num_pairs * 2]], dtype=np.int8)
        i_vals = iq_values[0::2].astype(np.float32)
        q_vals = iq_values[1::2].astype(np.float32)
        
        # Calculate Amplitudes: |H(k)| = sqrt(I^2 + Q^2)
        amplitudes = np.sqrt(i_vals**2 + q_vals**2)
        # Calculate Phases: phi(k) = atan2(Q, I)
        phases = np.arctan2(q_vals, i_vals)
        
        return {
            "timestamp_us": timestamp_us,
            "rssi": rssi,
            "noise_floor": noise_floor,
            "channel": channel,
            "bandwidth": bandwidth,
            "sig_mode": sig_mode,
            "antenna": antenna,
            "num_subcarriers": num_pairs,
            "amplitudes": amplitudes,
            "phases": phases,
            "raw_i": i_vals,
            "raw_q": q_vals
        }
    except Exception as e:
        return None

def main():
    parser = argparse.ArgumentParser(description="ESP32 Wi-Fi CSI Serial Receiver")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port (e.g. COM3, COM4, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--output", type=str, default="", help="Path to output CSV file")
    parser.add_argument("--label-presence", type=int, default=1, help="Ground truth presence label (0=empty, 1=present)")
    parser.add_argument("--label-activity", type=str, default="standing", help="Activity label (empty, standing, walking, sitting)")
    parser.add_argument("--label-zone", type=str, default="A1", help="Room Zone (e.g., A1, A2, B1)")
    parser.add_argument("--label-x", type=float, default=1.0, help="Ground truth X coordinate in meters")
    parser.add_argument("--label-y", type=float, default=1.0, help="Ground truth Y coordinate in meters")
    parser.add_argument("--label-direction", type=str, default="stationary", help="Movement direction (stationary, north, south, east, west)")
    parser.add_argument("--session-id", type=str, default="session_1", help="Data collection session identifier")
    parser.add_argument("--duration", type=int, default=0, help="Duration to collect in seconds (0 = infinite)")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  ESP32-S3 CSI Real-Time Serial Receiver & Logger")
    print(f"  Port: {args.port} | Baud: {args.baud}")
    print(f"  Labels: Zone={args.label_zone}, Coord=({args.label_x}, {args.label_y})m, Act={args.label_activity}")
    print("=" * 60)
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        ser.reset_input_buffer()
        print(f"[+] Successfully opened {args.port}. Listening for CSI streams...\n")
    except Exception as e:
        print(f"[!] Error opening serial port {args.port}: {e}")
        print("    Tip: Check device manager for correct COM port number.")
        sys.exit(1)
        
    csv_file = None
    csv_writer = None
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        csv_file = open(args.output, "w", buffering=1, encoding="utf-8")
        # Header dynamically written on first packet
    
    packet_count = 0
    start_time = time.time()
    last_stat_time = time.time()
    last_packet_count = 0
    header_written = False
    
    try:
        while True:
            line_bytes = ser.readline()
            if not line_bytes:
                continue
            
            try:
                line_str = line_bytes.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            
            # Print STATS notifications from ESP32
            if line_str.startswith("STATS,"):
                print(f" [ESP32 Hardware] {line_str}")
                continue
                
            parsed = parse_csi_line(line_str)
            if parsed is None:
                continue
                
            packet_count += 1
            now = time.time()
            
            # Write to CSV if enabled
            if csv_file:
                n_sub = parsed["num_subcarriers"]
                if not header_written:
                    headers = [
                        "session_id", "timestamp_us", "rssi", "noise_floor", "channel",
                        "bandwidth", "sig_mode", "antenna", "num_subcarriers",
                        "label_presence", "label_activity", "label_zone", "label_x", "label_y", "label_direction"
                    ]
                    # Append I/Q and Amplitude column headers
                    for i in range(n_sub):
                        headers.extend([f"amp_{i}", f"phase_{i}", f"real_{i}", f"imag_{i}"])
                    csv_file.write(",".join(headers) + "\n")
                    header_written = True
                
                row = [
                    args.session_id,
                    str(parsed["timestamp_us"]),
                    str(parsed["rssi"]),
                    str(parsed["noise_floor"]),
                    str(parsed["channel"]),
                    str(parsed["bandwidth"]),
                    str(parsed["sig_mode"]),
                    str(parsed["antenna"]),
                    str(n_sub),
                    str(args.label_presence),
                    args.label_activity,
                    args.label_zone,
                    str(args.label_x),
                    str(args.label_y),
                    args.label_direction
                ]
                for i in range(n_sub):
                    row.extend([
                        f"{parsed['amplitudes'][i]:.3f}",
                        f"{parsed['phases'][i]:.3f}",
                        f"{parsed['raw_i'][i]:.1f}",
                        f"{parsed['raw_q'][i]:.1f}"
                    ])
                csv_file.write(",".join(row) + "\n")
            
            # Display real-time throughput status every 1.0 second
            if now - last_stat_time >= 1.0:
                elapsed = now - last_stat_time
                rate = (packet_count - last_packet_count) / elapsed
                avg_amp = np.mean(parsed["amplitudes"])
                amp_var = np.var(parsed["amplitudes"])
                print(f"[RX Rate: {rate:5.1f} pkts/sec] | Total: {packet_count:6d} | RSSI: {parsed['rssi']:3d} dBm | Subcarriers: {parsed['num_subcarriers']:2d} | Mean Amp: {avg_amp:5.2f} | Var: {amp_var:6.2f}")
                last_stat_time = now
                last_packet_count = packet_count
                
            if args.duration > 0 and (now - start_time) >= args.duration:
                print(f"\n[+] Duration of {args.duration}s reached. Stopping capture.")
                break
                
    except KeyboardInterrupt:
        print("\n[*] Capture interrupted by user.")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()
            print(f"[+] Dataset saved to: {args.output}")
        print(f"[+] Total packets collected: {packet_count}")

if __name__ == "__main__":
    main()
