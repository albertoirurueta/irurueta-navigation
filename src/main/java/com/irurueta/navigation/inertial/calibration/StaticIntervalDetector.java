package com.irurueta.navigation.inertial.calibration;

/**
 * This detector is in charge of determining when a static period of
 * IMU measurements starts and finishes.
 * Statis periods, are periods of time where the device is considered
 * to remain static (no movement applied to it).
 */
public class StaticIntervalDetector {

    private double mThreshold;

    private int mWindowSize;
}
