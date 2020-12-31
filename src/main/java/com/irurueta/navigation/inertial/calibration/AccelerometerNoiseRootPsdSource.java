package com.irurueta.navigation.inertial.calibration;

/**
 * Defines a source for estimated accelerometer noise root PSD (Power Spectral Density).
 */
public interface AccelerometerNoiseRootPsdSource {

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    double getAccelerometerBaseNoiseLevelRootPsd();
}
