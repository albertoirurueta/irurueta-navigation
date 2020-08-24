package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.LockedException;

public class IMUEasyCalibratorIntervalDetector {

    /**
     * Initial number of samples to be measured to obtain level of accelerometer noise.
     */
    private static final int DEFAULT_INITIAL_SAMPLES = 2500;

    /**
     * Estimates initial accelerometer noise level.
     */
    private IMUNoiseEstimator mNoiseEstimator = new IMUNoiseEstimator();

    private double mTimeInterval = IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS;

    private int mInitialSamples = DEFAULT_INITIAL_SAMPLES;

    IMUEasyCalibratorIntervalDetector() {
        try {
            mNoiseEstimator.setTimeInterval(mTimeInterval);
            mNoiseEstimator.setTotalSamples(mInitialSamples);
        } catch (LockedException ignore) {
        }
    }
}
