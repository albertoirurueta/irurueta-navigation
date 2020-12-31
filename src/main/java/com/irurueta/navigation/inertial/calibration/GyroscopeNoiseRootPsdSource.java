package com.irurueta.navigation.inertial.calibration;

/**
 * Defines a source for estimated gyroscope noise root PSD (Power Spectral Density).
 */
public interface GyroscopeNoiseRootPsdSource {

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    double getGyroscopeBaseNoiseLevelRootPsd();
}
