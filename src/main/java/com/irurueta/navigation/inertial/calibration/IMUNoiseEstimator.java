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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * This class estimates IMU accelerometer and gyroscope noise variances and PSD's (Power
 * Spectral Densities) along with their average values.
 * This estimator must be used when the body where the IMU is attached remains static
 * on the same position with zero velocity while capturing data.
 * To compute PSD's, this estimator assumes that IMU samples are obtained at a constant
 * provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, IMU sampling rate average can be estimated using
 * {@link IMUTimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * This estimator is very similar to {@link IMUBiasEstimator} except that location and
 * orientation is not required, hence noise levels can be correctly determined, but
 * average levels do not necessarily math IMU bias values.
 */
public class IMUNoiseEstimator {
    /**
     * Default total samples to be processed.
     */
    public static final int DEFAULT_TOTAL_SAMPLES = 100000;

    /**
     * Default time interval between body kinematics samples expressed in seconds (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Total samples to be processed to finish estimation.
     */
    private int mTotalSamples = DEFAULT_TOTAL_SAMPLES;

    /**
     * Time interval expressed in seconds (s) between body kinematics samples.
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Listener to handle events raised by this estimator.
     */
    private IMUNoiseEstimatorListener mListener;

    /**
     * Last provided body kinematics values.
     */
    private BodyKinematics mLastBodyKinematics;

    /**
     * Contains estimated average of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mAvgFx;

    /**
     * Contains estimated average of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mAvgFy;

    /**
     * Contains estimated average of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mAvgFz;

    /**
     * Contains estimated average of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateX;

    /**
     * Contains estimated average of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateY;

    /**
     * Contains estimated average of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mAvgAngularRateZ;

    /**
     * Contains estimated variance of x coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     */
    private double mVarianceFx;

    /**
     * Contains estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     */
    private double mVarianceFy;

    /**
     * Contains estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     */
    private double mVarianceFz;

    /**
     * Contains estimated variance of x coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateX;

    /**
     * Contains estimated variance of y coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateY;

    /**
     * Contains estimated variance of z coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double mVarianceAngularRateZ;

    /**
     * Number of processed body kinematics samples.
     */
    private int mNumberOfProcessedSamples;

    /**
     * Number of processed timestamp samples plus one.
     */
    private int mNumberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public IMUNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public IMUNoiseEstimator(final IMUNoiseEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUNoiseEstimator(final int totalSamples) {
        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUNoiseEstimator(final int totalSamples,
                             final IMUNoiseEstimatorListener listener) {
        this(totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUNoiseEstimator(final double timeInterval) {
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUNoiseEstimator(final Time timeInterval) {
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUNoiseEstimator(final double timeInterval,
                             final IMUNoiseEstimatorListener listener) {
        this(timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUNoiseEstimator(final Time timeInterval,
                             final IMUNoiseEstimatorListener listener) {
        this(timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics (IMU acceleration +
     *                     gyroscope) samples expressed in seconds (s).
     * @throws IllegalArgumentException if provided total samples is zero or negative
     *                                  or if provided time interval is negative.
     */
    public IMUNoiseEstimator(final int totalSamples, final double timeInterval) {
        this(totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics (IMU acceleration +
     *                     gyroscope) samples.
     * @throws IllegalArgumentException if provided total samples is zero or negative
     *                                  or if provided time interval is negative.
     */
    public IMUNoiseEstimator(final int totalSamples, final Time timeInterval) {
        this(totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics (IMU acceleration +
     *                     gyroscope) samples expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative
     *                                  or if provided time interval is negative.
     */
    public IMUNoiseEstimator(final int totalSamples, final double timeInterval,
                             final IMUNoiseEstimatorListener listener) {
        this(totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics (IMU acceleration +
     *                     gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative
     *                                  or if provided time interval is negative.
     */
    public IMUNoiseEstimator(final int totalSamples, final Time timeInterval,
                             final IMUNoiseEstimatorListener listener) {
        this(totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Gets total samples to be processed to finish estimation.
     *
     * @return total samples to be processed to finish estimation.
     */
    public int getTotalSamples() {
        return mTotalSamples;
    }

    /**
     * Sets total samples to be processed to finish estimation.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws LockedException if estimator is currently running.
     */
    public void setTotalSamples(final int totalSamples) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return mTimeInterval;
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        mTimeInterval = timeInterval;
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(mTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(mTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public IMUNoiseEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final IMUNoiseEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets last provided body kinematics values or null if not available.
     *
     * @return last provided body kinematics values or null.
     */
    public BodyKinematics getLastBodyKinematics() {
        return mLastBodyKinematics != null ?
                new BodyKinematics(mLastBodyKinematics) : null;
    }

    /**
     * Gets last provided body kinematics values.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastBodyKinematics(final BodyKinematics result) {
        if (mLastBodyKinematics != null) {
            mLastBodyKinematics.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed specific force.
     */
    public double getAvgFx() {
        return mAvgFx;
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed specific force.
     */
    public Acceleration getAvgFxAsAcceleration() {
        return new Acceleration(mAvgFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgFxAsAcceleration(final Acceleration result) {
        result.setValue(mAvgFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public double getAvgFy() {
        return mAvgFy;
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public Acceleration getAvgFyAsAcceleration() {
        return new Acceleration(mAvgFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgFyAsAcceleration(final Acceleration result) {
        result.setValue(mAvgFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public double getAvgFz() {
        return mAvgFz;
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public Acceleration getAvgFzAsAcceleration() {
        return new Acceleration(mAvgFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgFzAsAcceleration(final Acceleration result) {
        result.setValue(mAvgFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public double getAvgAngularRateX() {
        return mAvgAngularRateX;
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateXAsAngularSpeed() {
        return new AngularSpeed(mAvgAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateXAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mAvgAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed angular rate.
     */
    public double getAvgAngularRateY() {
        return mAvgAngularRateY;
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateYAsAngularSpeed() {
        return new AngularSpeed(mAvgAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateYAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mAvgAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed angular rate.
     */
    public double getAvgAngularRateZ() {
        return mAvgAngularRateZ;
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateZAsAngularSpeed() {
        return new AngularSpeed(mAvgAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateZAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mAvgAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of body kinematics.
     *
     * @return estimated average of body kinematics.
     */
    public BodyKinematics getAvgBodyKinematics() {
        final BodyKinematics result = new BodyKinematics();
        getAvgBodyKinematics(result);
        return result;
    }

    /**
     * Gets estimated average of body kinematics.
     *
     * @param result instance where estimated average of body kinematics will be stored.
     */
    public void getAvgBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(mAvgFx, mAvgFy, mAvgFz);
        result.setAngularRateCoordinates(
                mAvgAngularRateX, mAvgAngularRateY, mAvgAngularRateZ);
    }

    /**
     * Gets estimated variance of x coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of x coordinate of sensed specific force.
     */
    public double getVarianceFx() {
        return mVarianceFx;
    }

    /**
     * Gets estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of y coordinate of sensed specific force.
     */
    public double getVarianceFy() {
        return mVarianceFy;
    }

    /**
     * Gets estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of z coordinate of sensed specific force.
     */
    public double getVarianceFz() {
        return mVarianceFz;
    }

    /**
     * Gets estimated variance of x coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of x coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateX() {
        return mVarianceAngularRateX;
    }

    /**
     * Gets estimated variance of y coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of y coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateY() {
        return mVarianceAngularRateY;
    }

    /**
     * Gets estimated variance of z coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of z coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateZ() {
        return mVarianceAngularRateZ;
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of x coordinate of sensed specific force.
     */
    public double getStandardDeviationFx() {
        return Math.sqrt(mVarianceFx);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of x coordinate of sensed specific force.
     */
    public Acceleration getStandardDeviationFxAsAcceleration() {
        return new Acceleration(getStandardDeviationFx(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of x coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFxAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFx());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationFy() {
        return Math.sqrt(mVarianceFy);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationFyAsAcceleration() {
        return new Acceleration(getStandardDeviationFy(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of y coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFyAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFy());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationFz() {
        return Math.sqrt(mVarianceFz);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationFzAsAcceleration() {
        return new Acceleration(getStandardDeviationFz(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of z coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFzAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFz());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of x coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateX() {
        return Math.sqrt(mVarianceAngularRateX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of x coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateXAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateX(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of x coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateXAsAngularSpeed(
            final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateX());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of y coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateY() {
        return Math.sqrt(mVarianceAngularRateY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of y coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateYAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateY(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of y coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateYAsAngularSpeed(
            final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateY());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of z coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateZ() {
        return Math.sqrt(mVarianceAngularRateZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of z coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateZAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateZ(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of z coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateZAsAngularSpeed(
            final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateZ());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @return a body kinematics instance containing standard deviation values.
     */
    public BodyKinematics getStandardDeviationsAsBodyKinematics() {
        return new BodyKinematics(getStandardDeviationFx(),
                getStandardDeviationFy(),
                getStandardDeviationFz(),
                getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @param result instance where data will be stored.
     */
    public void getStandardDeviationsAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(getStandardDeviationFx(),
                getStandardDeviationFy(), getStandardDeviationFz());
        result.setAngularRateCoordinates(getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on x axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on x axis.
     */
    public double getPSDFx() {
        return mVarianceFx * mTimeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on y axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on y axis.
     */
    public double getPSDFy() {
        return mVarianceFy * mTimeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on z axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on z axis.
     */
    public double getPSDFz() {
        return mVarianceFz * mTimeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on x axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on x axis.
     */
    public double getPSDAngularRateX() {
        return mVarianceAngularRateX * mTimeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on y axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on y axis.
     */
    public double getPSDAngularRateY() {
        return mVarianceAngularRateY * mTimeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on z axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on z axis.
     */
    public double getPSDAngularRateZ() {
        return mVarianceAngularRateZ * mTimeInterval;
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on x axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on x axis.
     */
    public double getRootPSDFx() {
        return Math.sqrt(getPSDFx());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on y axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on y axis.
     */
    public double getRootPSDFy() {
        return Math.sqrt(getPSDFy());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on z axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on z axis.
     */
    public double getRootPSDFz() {
        return Math.sqrt(getPSDFz());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on x axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on x axis.
     */
    public double getRootPSDAngularRateX() {
        return Math.sqrt(getPSDAngularRateX());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on y axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on y axis.
     */
    public double getRootPSDAngularRateY() {
        return Math.sqrt(getPSDAngularRateY());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on z axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on z axis.
     */
    public double getRootPSDAngularRateZ() {
        return Math.sqrt(getPSDAngularRateZ());
    }

    /**
     * Gets average accelerometer noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (m^2/s^-3).
     *
     * @return average accelerometer noise PSD.
     */
    public double getAccelerometerNoisePSD() {
        return (getPSDFx() + getPSDFy() + getPSDFz()) / 3.0;
    }

    /**
     * Gets average accelerometer noise root PSD (Power Spectral Density) among
     * x,y,z components expressed as (m * s^-1.5).
     *
     * @return average accelerometer noise root PSD.
     */
    public double getAccelerometerNoiseRootPSD() {
        return Math.sqrt(getAccelerometerNoisePSD());
    }

    /**
     * Gets average gyroscope noise PSD (Power Spectral Density) among
     * x,y,z components expressed in (rad^2/s).
     *
     * @return average gyroscope noise PSD.
     */
    public double getGyroNoisePSD() {
        return (getPSDAngularRateX() + getPSDAngularRateY() + getPSDAngularRateZ())
                / 3.0;
    }

    /**
     * Gets average gyroscope noise root PSD (Power Spectral Density) among
     * x,y,z components expressed in (rad * s^-0.5).
     *
     * @return average gyroscope noise root PSD.
     */
    public double getGyroNoiseRootPSD() {
        return Math.sqrt(getGyroNoisePSD());
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return mNumberOfProcessedSamples;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether estimator has finished the estimation.
     *
     * @return true if estimator has finished, false otherwise.
     */
    public boolean isFinished() {
        return mNumberOfProcessedSamples == mTotalSamples;
    }

    /**
     * Adds a sample of body kinematics (accelerometer + gyroscope readings) obtained
     * from an IMU.
     * If estimator is already finished, provided sample will be ignored.
     *
     * @param kinematics kinematics instance to be added and processed.
     * @return true if provided kinematics instance has been processed, false if it has
     * been ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addBodyKinematics(final BodyKinematics kinematics)
            throws LockedException {

        if (mRunning) {
            throw new LockedException();
        }

        if (isFinished()) {
            return true;
        }

        mRunning = true;

        if (mLastBodyKinematics == null && mListener != null) {
            mListener.onStart(this);
        }

        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();
        final double angularRateX = kinematics.getAngularRateX();
        final double angularRateY = kinematics.getAngularRateY();
        final double angularRateZ = kinematics.getAngularRateZ();

        // compute averages
        mAvgFx = mAvgFx * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + fx / (double) mNumberOfProcessedSamplesPlusOne;
        mAvgFy = mAvgFy * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + fy / (double) mNumberOfProcessedSamplesPlusOne;
        mAvgFz = mAvgFz * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + fz / (double) mNumberOfProcessedSamplesPlusOne;

        mAvgAngularRateX = mAvgAngularRateX * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + angularRateX / (double) mNumberOfProcessedSamplesPlusOne;
        mAvgAngularRateY = mAvgAngularRateY * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + angularRateY / (double) mNumberOfProcessedSamplesPlusOne;
        mAvgAngularRateZ = mAvgAngularRateZ * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + angularRateZ / (double) mNumberOfProcessedSamplesPlusOne;

        // compute variances
        final double diffFx = fx - mAvgFx;
        final double diffFy = fy - mAvgFy;
        final double diffFz = fz - mAvgFz;
        final double diffAngularRateX = angularRateX - mAvgAngularRateX;
        final double diffAngularRateY = angularRateY - mAvgAngularRateY;
        final double diffAngularRateZ = angularRateZ - mAvgAngularRateZ;

        final double diffFx2 = diffFx * diffFx;
        final double diffFy2 = diffFy * diffFy;
        final double diffFz2 = diffFz * diffFz;
        final double diffAngularRateX2 = diffAngularRateX * diffAngularRateX;
        final double diffAngularRateY2 = diffAngularRateY * diffAngularRateY;
        final double diffAngularRateZ2 = diffAngularRateZ * diffAngularRateZ;

        mVarianceFx = mVarianceFx * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFx2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceFy = mVarianceFy * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFy2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceFz = mVarianceFz * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFz2 / (double) mNumberOfProcessedSamplesPlusOne;

        mVarianceAngularRateX = mVarianceAngularRateX
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateX2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceAngularRateY = mVarianceAngularRateY
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateY2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceAngularRateZ = mVarianceAngularRateZ
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateZ2 / (double) mNumberOfProcessedSamplesPlusOne;

        mLastBodyKinematics = kinematics;

        mNumberOfProcessedSamples++;
        mNumberOfProcessedSamplesPlusOne++;

        if (mListener != null) {
            mListener.onBodyKinematicsAdded(this);
        }

        mRunning = false;

        if (isFinished() && mListener != null) {
            mListener.onFinish(this);
        }

        return true;
    }

    /**
     * Resets current estimator.
     *
     * @return true if estimator was successfully reset, false if no reset was needed.
     * @throws LockedException if estimator is currently running.
     */
    public boolean reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mNumberOfProcessedSamples == 0) {
            return false;
        }

        mRunning = true;
        mLastBodyKinematics = null;
        mAvgFx = 0.0;
        mAvgFy = 0.0;
        mAvgFz = 0.0;
        mAvgAngularRateX = 0.0;
        mAvgAngularRateY = 0.0;
        mAvgAngularRateZ = 0.0;
        mVarianceFx = 0.0;
        mVarianceFy = 0.0;
        mVarianceFz = 0.0;
        mVarianceAngularRateX = 0.0;
        mVarianceAngularRateY = 0.0;
        mVarianceAngularRateZ = 0.0;
        mNumberOfProcessedSamples = 0;
        mNumberOfProcessedSamplesPlusOne = 1;

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;

        return true;
    }
}
