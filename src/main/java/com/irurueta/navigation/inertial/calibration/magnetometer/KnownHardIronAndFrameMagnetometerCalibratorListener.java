package com.irurueta.navigation.inertial.calibration.magnetometer;

/**
 * Contains listener for magnetometer least squares calibrators when
 * frame (position, velocity and orientation) and bias is known for all
 * measurements.
 *
 * @param <T> a calibrator type.
 */
public interface KnownHardIronAndFrameMagnetometerCalibratorListener<T
        extends KnownHardIronAndFrameMagnetometerCalibrator<?, ?>> {

    /**
     * Called when calibration starts.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateStart(final T calibrator);

    /**
     * Called when calibration ends.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateEnd(final T calibrator);
}
