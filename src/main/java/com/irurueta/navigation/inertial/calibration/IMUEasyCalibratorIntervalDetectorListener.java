package com.irurueta.navigation.inertial.calibration;

public interface IMUEasyCalibratorIntervalDetectorListener {

    void onInitialIntervalCompleted(
            final IMUEasyCalibratorIntervalDetector detector,
            final double accelerometerPSD, final double accelerometerVariance);

    void onError(final IMUEasyCalibratorIntervalDetector detector);

    void onReset(final IMUEasyCalibratorIntervalDetector detector);
}
