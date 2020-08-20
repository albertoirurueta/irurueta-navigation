package com.irurueta.navigation.inertial.calibration.accelerometer;

/**
 * Contains listener for accelerometer calibrator when the same unknown position is used
 * for all measurements, bias is known and orientation is unknown for each measurement.
 *
 * @param <C> a calibrator type.
 */
public interface BaseBiasGravityNormAccelerometerCalibratorListener<C> {

    /**
     * Called when calibration starts.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateStart(final C calibrator);

    /**
     * Called when calibration ends.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateEnd(final C calibrator);
}
