#!/usr/bin/env python3
"""
features/extract_features.py — Feature Extraction Engine for Wi-Fi CSI Sensing

Extracts time-domain, frequency-domain, and spatial multipath features
from windowed CSI amplitude & phase matrices.
"""

import numpy as np
from scipy import stats
from scipy.fft import rfft, rfftfreq

def extract_window_features(window_amps: np.ndarray, fs: float = 100.0) -> np.ndarray:
    """
    Extracts comprehensive feature vector from a single CSI window.
    
    Args:
        window_amps: 2D numpy array [Time_samples, Subcarriers] (e.g. [100, 56])
        fs: Sampling frequency (Hz)
        
    Returns:
        1D numpy array representing the engineered feature representation.
    """
    features = []
    
    # 1. Subcarrier Time-Series Statistics across time (axis=0)
    # Shape: [N_subcarriers]
    mean_amp = np.mean(window_amps, axis=0)
    var_amp = np.var(window_amps, axis=0)
    std_amp = np.std(window_amps, axis=0)
    mad_amp = np.mean(np.abs(window_amps - mean_amp), axis=0)
    
    features.extend(mean_amp)
    features.extend(var_amp)
    features.extend(std_amp)
    features.extend(mad_amp)
    
    # 2. Global Window Dynamics (across all subcarriers & time)
    global_variance = np.var(window_amps)
    global_mad = np.mean(np.abs(window_amps - np.mean(window_amps)))
    energy = np.sum(window_amps ** 2) / (window_amps.shape[0] * window_amps.shape[1])
    
    features.append(global_variance)
    features.append(global_mad)
    features.append(energy)
    
    # 3. Frequency-Domain Doppler & Spectral Energy
    # Sum across subcarriers to obtain total channel perturbation envelope
    envelope = np.sum(window_amps, axis=1)
    envelope = envelope - np.mean(envelope)  # remove DC
    
    fft_vals = np.abs(rfft(envelope))
    freqs = rfftfreq(len(envelope), d=1.0/fs)
    
    # Gait & Walking cadence band: 0.5 Hz - 2.5 Hz
    mask_gait = (freqs >= 0.5) & (freqs <= 2.5)
    gait_energy = np.sum(fft_vals[mask_gait] ** 2) if np.any(mask_gait) else 0.0
    
    # Fast motion band: 2.5 Hz - 6.0 Hz
    mask_fast = (freqs > 2.5) & (freqs <= 6.0)
    fast_energy = np.sum(fft_vals[mask_fast] ** 2) if np.any(mask_fast) else 0.0
    
    # Total spectral power and spectral entropy
    total_power = np.sum(fft_vals ** 2) + 1e-9
    norm_power = (fft_vals ** 2) / total_power
    spectral_entropy = -np.sum(norm_power * np.log2(norm_power + 1e-12))
    
    features.append(gait_energy)
    features.append(fast_energy)
    features.append(total_power)
    features.append(spectral_entropy)
    
    # 4. Spatial Subcarrier Correlation Profile
    # Cross-correlation between low-frequency and high-frequency subcarriers
    n_sub = window_amps.shape[1]
    if n_sub >= 4:
        low_sub_mean = np.mean(window_amps[:, :n_sub//4], axis=1)
        high_sub_mean = np.mean(window_amps[:, -n_sub//4:], axis=1)
        corr = np.corrcoef(low_sub_mean, high_sub_mean)[0, 1]
        features.append(0.0 if np.isnan(corr) else corr)
    else:
        features.append(0.0)
        
    return np.array(features, dtype=np.float32)

def batch_feature_extraction(windows: np.ndarray, fs: float = 100.0) -> np.ndarray:
    """
    Extracts features for a batch of windows.
    Input: [N_windows, Time_samples, Subcarriers]
    Output: [N_windows, N_features]
    """
    feat_list = [extract_window_features(w, fs=fs) for w in windows]
    return np.array(feat_list, dtype=np.float32)
