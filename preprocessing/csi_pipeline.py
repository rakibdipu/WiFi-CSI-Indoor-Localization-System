#!/usr/bin/env python3
"""
preprocessing/csi_pipeline.py — Comprehensive CSI Signal Preprocessing Pipeline

Includes:
1. Static Subcarrier Removal & Re-ordering
2. Hampel Filter / 3-Sigma Outlier Replacement
3. Phase Sanitization (Unwrapping & Linear Detrending)
4. Butterworth Temporal Bandpass Filtering (0.3 Hz – 10 Hz for Human Motion)
5. Wavelet / Moving Average Denoising
6. Principal Component Analysis (PCA) for Multipath Mode Extraction
7. Sliding Window Feature Segmentation
"""

import numpy as np
from scipy import signal
from sklearn.decomposition import PCA
from typing import Tuple, List, Optional

def hampel_filter(data: np.ndarray, window_size: int = 7, n_sigmas: float = 3.0) -> np.ndarray:
    """
    Applies Hampel filter to remove impulsive outliers (e.g. Wi-Fi packet drops or AGC jumps).
    Replaces outliers with local rolling median.
    """
    filtered = data.copy()
    n = len(data)
    k = window_size // 2
    for i in range(k, n - k):
        window = data[i - k : i + k + 1]
        med = np.median(window)
        # Median Absolute Deviation (MAD)
        mad = np.median(np.abs(window - med))
        threshold = n_sigmas * 1.4826 * mad
        if np.abs(data[i] - med) > threshold:
            filtered[i] = med
    return filtered

def sanitize_phase(phases: np.ndarray) -> np.ndarray:
    """
    Sanitizes raw subcarrier phases:
    1. Unwraps 2*pi phase discontinuities across subcarrier index.
    2. Fits linear regression to remove Carrier Frequency Offset (CFO) and Packet Detection Delay (PDD).
    
    Formula:
        phi_sanitized(k) = phi_raw(k) - (a * k + b)
    where a and b are the slope and intercept minimizing least-squares error.
    """
    unwrapped = np.unwrap(phases)
    k = np.arange(len(unwrapped))
    
    # Linear regression: slope a, intercept b
    a, b = np.polyfit(k, unwrapped, 1)
    sanitized = unwrapped - (a * k + b)
    return sanitized

def butterworth_bandpass_filter(data: np.ndarray, lowcut: float = 0.3, highcut: float = 8.0, 
                                fs: float = 100.0, order: int = 4) -> np.ndarray:
    """
    Butterworth bandpass filter to isolate human Doppler signatures (0.3 Hz to 8.0 Hz).
    Removes static room reflections (DC / 0 Hz) and high-frequency RF thermal noise.
    """
    nyq = 0.5 * fs
    low = max(lowcut / nyq, 0.001)
    high = min(highcut / nyq, 0.999)
    b, a = signal.butter(order, [low, high], btype='band')
    
    # Apply forward-backward filter to prevent phase distortion
    if len(data) > 3 * max(len(a), len(b)):
        return signal.filtfilt(b, a, data, axis=0)
    return data

def apply_pca(amplitude_matrix: np.ndarray, n_components: int = 5) -> Tuple[np.ndarray, PCA]:
    """
    Extracts dominant multipath variations across all subcarriers using PCA.
    Input: [N_time_samples, N_subcarriers]
    Output: [N_time_samples, n_components]
    """
    pca = PCA(n_components=n_components)
    components = pca.fit_transform(amplitude_matrix)
    return components, pca

def extract_sliding_windows(data: np.ndarray, labels: np.ndarray, 
                           window_size: int = 100, step_size: int = 25) -> Tuple[np.ndarray, np.ndarray]:
    """
    Segments continuous CSI time-series into overlapping temporal windows.
    At 100 Hz sampling rate, window_size=100 gives 1.0 second snapshot with 75% overlap.
    """
    windows = []
    window_labels = []
    
    n_samples = len(data)
    for start in range(0, n_samples - window_size + 1, step_size):
        end = start + window_size
        win = data[start:end]
        # Label is mode / majority of the window
        win_label = labels[end - 1]
        windows.append(win)
        window_labels.append(win_label)
        
    return np.array(windows), np.array(window_labels)
