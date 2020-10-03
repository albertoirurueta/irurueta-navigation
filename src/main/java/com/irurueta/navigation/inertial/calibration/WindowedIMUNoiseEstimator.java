package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyKinematics;

import java.util.LinkedList;

public class WindowedIMUNoiseEstimator {

    /**
     * Number of samples to keep within the window by default.
     * For an IMU generating 100 samples/second, this is equivalent to roughly
     * 1 second.
     * For an IMU generating 50 samples/second this is equivalent to roughly
     * 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = 101;

    /**
     * Minimum allowed window size.
     */
    public static final int MIN_WINDOW_SIZE = 2;

    /**
     * Default time interval between body kinematics samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Length of number of samples to keep within the window being processed.
     * Window size must always be odd (so that a center sample exists), and
     * must always be larger than allowed minimum value.
     */
    private int mWindowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Time interval expressed in seconds (s) between consecutive body kinematics
     * samples.
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Keeps the list of IMU samples that remain within the window.
     */
    private LinkedList<BodyKinematics> mWindowedSamples = new LinkedList<>();

    /**
     * Contains estimated average of x coordinate of accelerometer sensed
     * specific force within current window expressed in meters per squared
     * second (m/s^2).
     */
    private double mAvgFx;

    /**
     * Contains estimated average of y coordinate of accelerometer sensed
     * specific force within current window expressed in meters per squared
     * second (m/s^2).
     */
    private double mAvgFy;

    /**
     * Contains estimated average of z coordinate of accelerometer sensed
     * specific force within current window expressed in meters per squared
     * second (m/s^2).
     */
    private double mAvgFz;

    /**
     * Contains estimated average of x coordinate of gyroscope sensed angular
     * rate within current window expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateX;

    /**
     * Contains estimated average of y coordinate of gyroscope sensed angular
     * rate within current window expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateY;

    /**
     * Contains estimated average of z coordinate of gyroscope sensed angular
     * rate within current window expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateZ;

    /**
     * Contains estimated variance of x coordinate of accelerometer sensed
     * specific force within current window expressed in (m^2/s^4).
     */
    private double mVarianceFx;

    /**
     * Contains estimated variance of y coordinate of accelerometer sensed
     * specific force within current window expressed in (m^2/s^4).
     */
    private double mVarianceFy;

    /**
     * Contains estimated variance of z coordinate of accelerometer sensed
     * specific force within current window expressed in (m^2/s^4).
     */
    private double mVarianceFz;

    /**
     * Contains estimated variance of x coordinate of gyroscope sensed angular
     * rate within current window expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateX;

    /**
     * Contains estimated variance of y coordinate of gyroscope sensed angular
     * rate within current window expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateY;

    /**
     * Contains estimated variance of z coordinate of gyroscope sensed angular
     * rate within current window expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateZ;
}
