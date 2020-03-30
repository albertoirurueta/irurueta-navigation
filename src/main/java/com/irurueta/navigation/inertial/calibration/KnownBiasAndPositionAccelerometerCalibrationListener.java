package com.irurueta.navigation.inertial.calibration;

/**
 * Contains listener for accelerometer calibrator when the same position is known
 * for all measurements, bias is known and orientation is unknown.
 */
public interface KnownBiasAndPositionAccelerometerCalibrationListener {

    /**
     * Called when calibration starts.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateStart(final KnownBiasAndPositionAccelerometerCalibrator calibrator);

    /**
     * Called when calibration ends.
     *
     * @param calibrator calibrator that raised the event.
     */
    void onCalibrateEnd(final KnownBiasAndPositionAccelerometerCalibrator calibrator);
}
