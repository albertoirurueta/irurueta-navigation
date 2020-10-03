/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

public class IMUEasyCalibratorIntervalDetector {

    /**
     * Initial number of samples to be measured to obtain level of accelerometer noise.
     */
    private static final int DEFAULT_INITIAL_SAMPLES = 2500;

    /**
     * Default maximum threshold multiplier value.
     */
    private static final int DEFAULT_MAX_INITIAL_VARIANCE_THRESHOLD_MULTIPLIER = 10;

    /**
     * Estimates initial accelerometer noise level.
     */
    //private final IMUNoiseEstimator mInitialNoiseEstimator =
    //        new IMUNoiseEstimator();

    /**
     * Time internal between kinematics samples expressed in seconds (s).
     */
    private double mTimeInterval; // = IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Total samples to be processed during initial interval.
     */
    private int mInitialSamples = DEFAULT_INITIAL_SAMPLES;

    /**
     * Indicates whether initial interval has already been completed.
     */
    private boolean mInitialIntervalCompleted;

    /**
     * Estimated accelerometer variance during initial interval expressed in (m^2/s^4).
     */
    private double mInitialAccelerometerVariance;

    /**
     * Maximum allowed accelerometer variance during initial interval.
     * During initial interval IMU is supposed to be static. If variance
     * exceeds this value (because IMU is too noisy or a sudden
     * movement occurs), then the detector will notify an error and reset.
     * If this value is null, no limit is imposed on allowed initial
     * accelerometer variance.
     */
    private Double mMaxInitialAccelerometerVariance;

    /**
     * Maximum multiplier value for estimated initial variance to estimate best intervals.
     */
    private int mMaxInitialVarianceThresholdMultiplier = DEFAULT_MAX_INITIAL_VARIANCE_THRESHOLD_MULTIPLIER;

    /**
     * Indicates whether this detector is running.
     * The detector is considered to be running from the moment the first
     * kinematics measure is added until calibration is completed or fails,
     * or the detector is reset.
     */
    private boolean mRunning;

    /**
     * Listener to notify completion of intervals, calibration and errors.
     */
    private IMUEasyCalibratorIntervalDetectorListener mListener;

    /**
     * Constructor.
     */
    IMUEasyCalibratorIntervalDetector() {
        /*try {
            mInitialNoiseEstimator.setTimeInterval(mTimeInterval);
            mInitialNoiseEstimator.setTotalSamples(mInitialSamples);
            mInitialNoiseEstimator.setListener(new IMUNoiseEstimatorListener() {
                @Override
                public void onStart(IMUNoiseEstimator estimator) {

                }

                @Override
                public void onBodyKinematicsAdded(IMUNoiseEstimator estimator) {
                    mRunning = true;
                }

                @Override
                public void onFinish(IMUNoiseEstimator estimator) {
                    mInitialIntervalCompleted = true;
                    // Accelerometer Power Spectral Density (m^2/s^-3)
                    // Variance is m^2/s^4, PSD is variance * time interval
                    // psd = variance * timeInterval --> variance = psd / timeInterval
                    final double psd = estimator.getAccelerometerNoisePSD();
                    final double timeInterval = estimator.getTimeInterval();
                    mInitialAccelerometerVariance = psd / timeInterval;

                    if (mMaxInitialAccelerometerVariance != null
                            && mInitialAccelerometerVariance > mMaxInitialAccelerometerVariance) {
                        // variance exceeds maximum allowed value
                        try {
                            reset();
                        } catch (final LockedException ignore) {
                            // never happens
                        }
                        if (mListener != null) {
                            mListener.onError(
                                    IMUEasyCalibratorIntervalDetector.this);
                        }
                    } else {
                        if (mListener != null) {
                            mListener.onInitialIntervalCompleted(
                                    IMUEasyCalibratorIntervalDetector.this,
                                    psd, mInitialAccelerometerVariance);
                        }
                    }
                }

                @Override
                public void onReset(IMUNoiseEstimator estimator) {

                }
            });
        } catch (LockedException ignore) {
        }*/
    }

    /**
     * Indicates whether this detector is running or not.
     * The detector is considered to be running from the moment the first
     * kinematics measure is added until calibration is completed or fails,
     * or the detector is reset.
     *
     * @return true if the detector is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets time interval between kinematics samples expressed in
     * seconds (s).
     *
     * @return time interval between kinematics samples.
     */
    public double getTimeInterval() {
        return mTimeInterval;
    }

    /**
     * Sets time interval between kinematics samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between kinematics samples.
     * @throws LockedException          if detector is running.
     * @throws IllegalArgumentException if provided value is zero
     *                                  or negative.
     */
    public void setTimeInterval(final double timeInterval)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval <= 0.0) {
            throw new IllegalArgumentException();
        }

        mTimeInterval = timeInterval;
        //mInitialNoiseEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between kinematics samples.
     *
     * @return time interval between kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(mTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between kinematics samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(mTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between kinematics samples.
     *
     * @param timeInterval time interval between kinematics samples.
     * @throws LockedException          if detector is running.
     * @throws IllegalArgumentException if provided value is
     *                                  zero or negative.
     */
    public void setTimeInterval(final Time timeInterval)
            throws LockedException {
        setTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets total samples to be processed during initial interval.
     *
     * @return total samples to be processed during initial interval.
     */
    public int getInitialSamples() {
        return mInitialSamples;
    }

    /**
     * Sets total samples to be processed during initial interval.
     *
     * @param initialSamples total samples to be processed during initial
     *                       interval.
     * @throws LockedException          if detector is running.
     * @throws IllegalArgumentException if provided value is zero
     *                                  or negative.
     */
    public void setInitialSamples(final int initialSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mInitialSamples = initialSamples;
        //mInitialNoiseEstimator.setTotalSamples(initialSamples);
    }

    /**
     * Gets estimated initial accelerometer variance expressed
     * in (m^2/s^4).
     *
     * @return estimated initial accelerometer variance.
     */
    public double getInitialAccelerometerVariance() {
        return mInitialAccelerometerVariance;
    }

    /**
     * Gets maximum allowed accelerometer variance during initial interval.
     * During initial interval IMU is supposed to be static. If variance
     * exceeds this value (because IMU is too noisy or a sudden movement
     * occurs), then the detector will notify an error and reset.
     * If this value is null, no limit is imposed on allowed initial
     * accelerometer variance.
     *
     * @return maximum allowed accelerometer variance.
     */
    public Double getMaxInitialAccelerometerVariance() {
        return mMaxInitialAccelerometerVariance;
    }

    /**
     * Sets maximum allowed accelerometer variance during initial interval.
     * During initial interval IMU is supposed to be static. If variance
     * exceeds this value (because IMU is too noisy or a sudden movement
     * occurs), then the detector will notify an error and reset.
     * If this value is null, no limit is imposed on allowed initial
     * accelerometer variance.
     *
     * @param maximumInitialAccelerometerVariance maximum allowed
     *                                            accelerometer variance or
     *                                            null.
     * @throws LockedException if detector is running.
     */
    public void setMaximumInitialAccelerometerVariance(
            final Double maximumInitialAccelerometerVariance)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mMaxInitialAccelerometerVariance =
                maximumInitialAccelerometerVariance;
    }

    /**
     * Gets maximum multiplier value for estimated initial variance to
     * estimate best intervals.
     *
     * @return maximum multiplier value for estimated initial variance.
     */
    public int getMaxInitialVarianceThresholdMultiplier() {
        return mMaxInitialVarianceThresholdMultiplier;
    }

    /**
     * Sets maixmum multiplier value for estimated initial variance to
     * estimate best intervals.
     *
     * @param maxInitialVarianceThresholdMultiplier maximum multiplier value
     *                                              for estimated initial
     *                                              variance.
     * @throws LockedException if detector is running.
     * @throws IllegalArgumentException if provided value is less than 1.
     */
    public void setMaxInitialVarianceThresholdMultiplier(
            final int maxInitialVarianceThresholdMultiplier)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (maxInitialVarianceThresholdMultiplier < 1) {
            throw new IllegalArgumentException();
        }

        mMaxInitialVarianceThresholdMultiplier =
                maxInitialVarianceThresholdMultiplier;
    }

    /**
     * Gets listener to notify completion of intervals, calibration and
     * errors.
     *
     * @return listener to notify events.
     */
    public IMUEasyCalibratorIntervalDetectorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to notify completion of intervals, calibration and
     * errors.
     *
     * @param listener listener to notify events.
     * @throws LockedException if detector is running.
     */
    public void setListener(
            final IMUEasyCalibratorIntervalDetectorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Adds a body kinematics measure (accelerometer + gyroscope) to be
     * processed by this detector.
     *
     * @param kinematics body kinematics measure to be processed.
     * @throws LockedException if detector is running.
     */
    public void addKinematics(final BodyKinematics kinematics)
            throws LockedException {
        mRunning = true;
        if (!mInitialIntervalCompleted) {
            //mInitialNoiseEstimator.addBodyKinematics(kinematics);
        }
    }

    /**
     * Resets this detector.
     *
     * @throws LockedException if detector is running.
     */
    public void reset() throws LockedException {
        mRunning = false;
        //mInitialNoiseEstimator.reset();
        mInitialIntervalCompleted = false;
        mInitialAccelerometerVariance = 0.0;

        if (mListener != null) {
            mListener.onReset(this);
        }
    }
}
