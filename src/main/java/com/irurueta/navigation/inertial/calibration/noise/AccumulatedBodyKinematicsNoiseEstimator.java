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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;

/**
 * Estimates accumulated acceleration and angular speed noise variances and PSD's
 * (Power Spectral Densities) along with their average values.
 * This estimator must be used when the body where the accelerometer and gyroscope
 * are attached remains static on the same position with zero velocity and
 * constant (or zero) angular speed while capturing data.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not known, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations
 * (variance or standard deviation) can be safely used.
 */
public class AccumulatedBodyKinematicsNoiseEstimator
        implements AccelerometerNoiseRootPsdSource, GyroscopeNoiseRootPsdSource {

    /**
     * Default time interval between accelerometer samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS =
            AccumulatedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Listener to handle events raised by this estimator.
     */
    private AccumulatedBodyKinematicsNoiseEstimatorListener mListener;

    /**
     * Last provided body kinematics.
     */
    private BodyKinematics mLastBodyKinematics;

    /**
     * Accumulated acceleration estimator.
     */
    private final AccumulatedAccelerationTriadNoiseEstimator mAccelerationEstimator =
            new AccumulatedAccelerationTriadNoiseEstimator();

    /**
     * Accumulated angular speed estimator.
     */
    private final AccumulatedAngularSpeedTriadNoiseEstimator mAngularSpeedEstimator =
            new AccumulatedAngularSpeedTriadNoiseEstimator();

    /**
     * Indicates that estimator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public AccumulatedBodyKinematicsNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public AccumulatedBodyKinematicsNoiseEstimator(
            final AccumulatedBodyKinematicsNoiseEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Gets time interval between body kinematics samples expressed in
     * seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return mAccelerationEstimator.getTimeInterval();
    }

    /**
     * Sets time interval between body kinematics samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between body kinematic samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerationEstimator.setTimeInterval(timeInterval);
        mAngularSpeedEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between body kinematics samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return mAccelerationEstimator.getTimeIntervalAsTime();
    }

    /**
     * Gets time interval between body kinematics samples.
     *
     * @param result instance where body kinematics will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        mAccelerationEstimator.getTimeIntervalAsTime(result);
    }

    /**
     * Sets time interval between body kinematics samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerationEstimator.setTimeInterval(timeInterval);
        mAngularSpeedEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public AccumulatedBodyKinematicsNoiseEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final AccumulatedBodyKinematicsNoiseEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets last provided body kinematics or null if not available.
     *
     * @return last provided body kinematics or null.
     */
    public BodyKinematics getLastBodyKinematics() {
        return mLastBodyKinematics;
    }

    /**
     * Gets last provided body kinematics.
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
    public double getAvgSpecificForceX() {
        return mAccelerationEstimator.getAvgX();
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceXAsMeasurement() {
        return mAccelerationEstimator.getAvgXAsMeasurement();
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceXAsMeasurement(final Acceleration result) {
        mAccelerationEstimator.getAvgXAsMeasurement(result);
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public double getAvgSpecificForceY() {
        return mAccelerationEstimator.getAvgY();
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceYAsMeasurement() {
        return mAccelerationEstimator.getAvgYAsMeasurement();
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceYAsMeasurement(final Acceleration result) {
        mAccelerationEstimator.getAvgYAsMeasurement(result);
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public double getAvgSpecificForceZ() {
        return mAccelerationEstimator.getAvgZ();
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceZAsMeasurement() {
        return mAccelerationEstimator.getAvgZAsMeasurement();
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceZAsMeasurement(final Acceleration result) {
        mAccelerationEstimator.getAvgZAsMeasurement(result);
    }

    /**
     * Gets estimated average of accelerometer sensed specific force as a measurement
     * triad.
     *
     * @return average accelerometer triad.
     */
    public AccelerationTriad getAvgSpecificForceAsTriad() {
        return mAccelerationEstimator.getAvgTriad();
    }

    /**
     * Gets estimated average of accelerometer sensed specific force as a measurement
     * triad.
     *
     * @param result instance where average accelerometer triad will be stored.
     */
    public void getAvgSpecificForceAsTriad(final AccelerationTriad result) {
        mAccelerationEstimator.getAvgTriad(result);
    }

    /**
     * Gets norm of estimated average acceleration expressed in meters per squared
     * second (m/s^2). This value is independent of body orientation.
     *
     * @return norm of estimated average acceleration.
     */
    public double getAvgSpecificForceNorm() {
        return mAccelerationEstimator.getAvgNorm();
    }

    /**
     * Gets norm of estimated average acceleration within current window.
     *
     * @return norm of estimated average acceleration.
     */
    public Acceleration getAvgSpecificForceNormAsMeasurement() {
        return mAccelerationEstimator.getAvgNormAsMeasurement();
    }

    /**
     * Gets norm of estimated average acceleration.
     *
     * @param result instance where norm of estimated average acceleration will be stored.
     */
    public void getAvgSpecificForceNormAsMeasurement(final Acceleration result) {
        mAccelerationEstimator.getAvgNormAsMeasurement(result);
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should
     * never be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public double getAvgAngularRateX() {
        return mAngularSpeedEstimator.getAvgX();
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateXAsMeasurement() {
        return mAngularSpeedEstimator.getAvgXAsMeasurement();
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateXAsMeasurement(final AngularSpeed result) {
        mAngularSpeedEstimator.getAvgXAsMeasurement(result);
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
        return mAngularSpeedEstimator.getAvgY();
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateYAsMeasurement() {
        return mAngularSpeedEstimator.getAvgYAsMeasurement();
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateYAsMeasurement(final AngularSpeed result) {
        mAngularSpeedEstimator.getAvgYAsMeasurement(result);
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
        return mAngularSpeedEstimator.getAvgZ();
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateZAsMeasurement() {
        return mAngularSpeedEstimator.getAvgZAsMeasurement();
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateZAsMeasurement(final AngularSpeed result) {
        mAngularSpeedEstimator.getAvgZAsMeasurement(result);
    }

    /**
     * Gets estimated average of gyroscope sensed angular speed as a measurement
     * triad.
     *
     * @return average angular speed triad.
     */
    public AngularSpeedTriad getAvgAngularRateTriad() {
        return mAngularSpeedEstimator.getAvgTriad();
    }

    /**
     * Gets estimated average of gyroscope sensed angular speed as a measurement
     * triad.
     *
     * @param result instance where average angular speed triad will be stored.
     */
    public void getAvgAngularRateTriad(final AngularSpeedTriad result) {
        mAngularSpeedEstimator.getAvgTriad(result);
    }

    /**
     * Gets norm of estimated average angular speed expressed in radians per
     * second (rad/s). This value is independent of body orientation.
     *
     * @return norm of estimated average angular speed.
     */
    public double getAvgAngularRateNorm() {
        return mAngularSpeedEstimator.getAvgNorm();
    }

    /**
     * Gets norm of estimated average angular speed.
     * This value is independent of body orientation.
     *
     * @return norm of estimated average angular speed.
     */
    public AngularSpeed getAvgAngularRateNormAsMeasurement() {
        return mAngularSpeedEstimator.getAvgNormAsMeasurement();
    }

    /**
     * Gets norm of estimated average angular speed.
     * This value is independent of body orientation.
     *
     * @param result instance where norm of estimated average angular speed will be stored.
     */
    public void getAvgAngularRateNormAsMeasurement(final AngularSpeed result) {
        mAngularSpeedEstimator.getAvgNormAsMeasurement(result);
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
        final double avgFx = mAccelerationEstimator.getAvgX();
        final double avgFy = mAccelerationEstimator.getAvgY();
        final double avgFz = mAccelerationEstimator.getAvgZ();

        final double avgWx = mAngularSpeedEstimator.getAvgX();
        final double avgWy = mAngularSpeedEstimator.getAvgY();
        final double avgWz = mAngularSpeedEstimator.getAvgZ();

        result.setSpecificForceCoordinates(avgFx, avgFy, avgFz);
        result.setAngularRateCoordinates(avgWx, avgWy, avgWz);
    }

    /**
     * Gets estimated variance of x coordinate of accelerometer sensed specific force
     * expressed in (m^2/s^4).
     *
     * @return estimated variance of x coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceX() {
        return mAccelerationEstimator.getVarianceX();
    }

    /**
     * Gets estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of y coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceY() {
        return mAccelerationEstimator.getVarianceY();
    }

    /**
     * Gets estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of z coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceZ() {
        return mAccelerationEstimator.getVarianceZ();
    }

    /**
     * Gets estimated variance of x coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of x coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateX() {
        return mAngularSpeedEstimator.getVarianceX();
    }

    /**
     * Gets estimated variance of y coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of y coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateY() {
        return mAngularSpeedEstimator.getVarianceY();
    }

    /**
     * Gets estimated variance of z coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of z coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateZ() {
        return mAngularSpeedEstimator.getVarianceZ();
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of x coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceX() {
        return mAccelerationEstimator.getStandardDeviationX();
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of x coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceXAsMeasurement() {
        return mAccelerationEstimator.getStandardDeviationXAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of x
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceXAsMeasurement(
            final Acceleration result) {
        mAccelerationEstimator.getStandardDeviationXAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceY() {
        return mAccelerationEstimator.getStandardDeviationY();
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceYAsMeasurement() {
        return mAccelerationEstimator.getStandardDeviationYAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of y
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceYAsMeasurement(
            final Acceleration result) {
        mAccelerationEstimator.getStandardDeviationYAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceZ() {
        return mAccelerationEstimator.getStandardDeviationZ();
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceZAsMeasurement() {
        return mAccelerationEstimator.getStandardDeviationZAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of z
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceZAsMeasurement(
            final Acceleration result) {
        mAccelerationEstimator.getStandardDeviationZAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation triad of accelerometer measurements.
     *
     * @return estimated standard deviation triad of accelerometer measurements.
     */
    public AccelerationTriad getStandardDeviationSpecificForceTriad() {
        return mAccelerationEstimator.getStandardDeviationTriad();
    }

    /**
     * Gets estimated standard deviation triad of accelerometer measurements.
     *
     * @param result instance where estimated standard deviation triad of
     *               accelerometer measurements will be stored.
     */
    public void getStandardDeviationSpecificForceTriad(
            final AccelerationTriad result) {
        mAccelerationEstimator.getStandardDeviationTriad(result);
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements
     * expressed in meters per squared second (m/s^2).
     *
     * @return norm of estimated standard deviation of accelerometer
     * measurements.
     */
    public double getStandardDeviationSpecificForceNorm() {
        return mAccelerationEstimator.getStandardDeviationNorm();
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements.
     *
     * @return norm of estimated standard deviation of measurements.
     */
    public Acceleration getStandardDeviationSpecificForceNormAsMeasurement() {
        return mAccelerationEstimator.getStandardDeviationNormAsMeasurement();
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements.
     *
     * @param result instance where norm of estimated standard deviation will be
     *               stored.
     */
    public void getStandardDeviationSpecificForceNormAsMeasurement(
            final Acceleration result) {
        mAccelerationEstimator.getStandardDeviationNormAsMeasurement(
                result);
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements expressed in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation coordinates.
     */
    public double getAverageStandardDeviationSpecificForce() {
        return mAccelerationEstimator.getAverageStandardDeviation();
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements.
     *
     * @return average of estimated standard deviation coordinates.
     */
    public Acceleration getAverageStandardDeviationSpecificForceAsMeasurement() {
        return mAccelerationEstimator.getAverageStandardDeviationAsMeasurement();
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements.
     *
     * @param result instance where average of estimated standard deviation coordinates
     *               will be stored.
     */
    public void getAverageStandardDeviationSpecificForceAsMeasurement(
            final Acceleration result) {
        mAccelerationEstimator.getAverageStandardDeviationAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of x coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateX() {
        return mAngularSpeedEstimator.getStandardDeviationX();
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope.
     *
     * @return estimated standard deviation of x coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateXAsMeasurement() {
        return mAngularSpeedEstimator.getStandardDeviationXAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope.
     *
     * @param result estimated standard deviation of x coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateXAsMeasurement(
            final AngularSpeed result) {
        mAngularSpeedEstimator.getStandardDeviationXAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of y coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateY() {
        return mAngularSpeedEstimator.getStandardDeviationY();
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope.
     *
     * @return estimated standard deviation of y coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateYAsMeasurement() {
        return mAngularSpeedEstimator.getStandardDeviationYAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope.
     *
     * @param result estimated standard deviation of y coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateYAsMeasurement(
            final AngularSpeed result) {
        mAngularSpeedEstimator.getStandardDeviationYAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of z coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateZ() {
        return mAngularSpeedEstimator.getStandardDeviationZ();
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope.
     *
     * @return estimated standard deviation of z coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateZAsMeasurement() {
        return mAngularSpeedEstimator.getStandardDeviationZAsMeasurement();
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope.
     *
     * @param result estimated standard deviation of z coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateZAsMeasurement(
            final AngularSpeed result) {
        mAngularSpeedEstimator.getStandardDeviationZAsMeasurement(result);
    }

    /**
     * Gets estimated standard deviation triad of angular speed measurements.
     *
     * @return estimated standard deviation triad of angular speed measurements.
     */
    public AngularSpeedTriad getStandardDeviationAngularSpeedTriad() {
        return mAngularSpeedEstimator.getStandardDeviationTriad();
    }

    /**
     * Gets estimated standard deviation triad of angular speed measurements.
     *
     * @param result instance where estimated standard deviation triad of
     *               gyroscope measurements will be stored.
     */
    public void getStandardDeviationAngularSpeedTriad(
            final AngularSpeedTriad result) {
        mAngularSpeedEstimator.getStandardDeviationTriad(result);
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements
     * expressed in radians per second (rad/s).
     *
     * @return norm of estimated standard deviation of gyroscope
     * measurements.
     */
    public double getStandardDeviationAngularSpeedNorm() {
        return mAngularSpeedEstimator.getStandardDeviationNorm();
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements.
     *
     * @return norm of estimated standard deviation of measurements.
     */
    public AngularSpeed getStandardDeviationAngularSpeedNormAsMeasurement() {
        return mAngularSpeedEstimator.getStandardDeviationNormAsMeasurement();
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements.
     *
     * @param result instance where norm of estimated standard deviation will be
     *               stored.
     */
    public void getStandardDeviationAngularSpeedNormAsMeasurement(
            final AngularSpeed result) {
        mAngularSpeedEstimator.getStandardDeviationNormAsMeasurement(result);
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements expressed in radians per second (rad/s).
     *
     * @return average of estimated standard deviation coordinates.
     */
    public double getAverageStandardDeviationAngularSpeed() {
        return mAngularSpeedEstimator.getAverageStandardDeviation();
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements.
     *
     * @return average of estimated standard deviation coordinates.
     */
    public AngularSpeed getAverageStandardDeviationAngularSpeedAsMeasurement() {
        return mAngularSpeedEstimator.getAverageStandardDeviationAsMeasurement();
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements.
     *
     * @param result instance where average of estimated standard deviation coordinates
     *               will be stored.
     */
    public void getAverageStandardDeviationAngularSpeedAsMeasurement(
            final AngularSpeed result) {
        mAngularSpeedEstimator.getAverageStandardDeviationAsMeasurement(
                result);
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @return a body kinematics instance containing standard deviation values.
     */
    public BodyKinematics getStandardDeviationAsBodyKinematics() {
        return new BodyKinematics(getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ(),
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
    public void getStandardDeviationAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ());
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
    public double getSpecificForcePsdX() {
        return mAccelerationEstimator.getPsdX();
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on y axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on y axis.
     */
    public double getSpecificForcePsdY() {
        return mAccelerationEstimator.getPsdY();
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on z axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on z axis.
     */
    public double getSpecificForcePsdZ() {
        return mAccelerationEstimator.getPsdZ();
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on x axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on x axis.
     */
    public double getAngularRatePsdX() {
        return mAngularSpeedEstimator.getPsdX();
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on y axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on y axis.
     */
    public double getAngularRatePsdY() {
        return mAngularSpeedEstimator.getPsdY();
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on z axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on z axis.
     */
    public double getAngularRatePsdZ() {
        return mAngularSpeedEstimator.getPsdZ();
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on x axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on x axis.
     */
    public double getSpecificForceRootPsdX() {
        return mAccelerationEstimator.getRootPsdX();
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on y axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on y axis.
     */
    public double getSpecificForceRootPsdY() {
        return mAccelerationEstimator.getRootPsdY();
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on z axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on z axis.
     */
    public double getSpecificForceRootPsdZ() {
        return mAccelerationEstimator.getRootPsdZ();
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on x axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on x axis.
     */
    public double getAngularRateRootPsdX() {
        return mAngularSpeedEstimator.getRootPsdX();
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on y axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on y axis.
     */
    public double getAngularRateRootPsdY() {
        return mAngularSpeedEstimator.getRootPsdY();
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on z axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on z axis.
     */
    public double getAngularRateRootPsdZ() {
        return mAngularSpeedEstimator.getRootPsdZ();
    }

    /**
     * Gets average accelerometer noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (m^2/s^-3).
     *
     * @return average accelerometer noise PSD.
     */
    public double getAvgSpecificForceNoisePsd() {
        return mAccelerationEstimator.getAvgNoisePsd();
    }

    /**
     * Gets norm of noise root PSD (Power Spectral Density) among x,y,z
     * components expressed as (m * s^-1.5).
     *
     * @return norm of noise root PSD.
     */
    public double getSpecificForceNoiseRootPsdNorm() {
        return mAccelerationEstimator.getNoiseRootPsdNorm();
    }

    /**
     * Gets average gyroscope noise PSD (Power Spectral Density) among
     * x,y,z components expressed in (rad^2/s).
     *
     * @return average gyroscope noise PSD.
     */
    public double getAvgAngularRateNoisePsd() {
        return mAngularSpeedEstimator.getAvgNoisePsd();
    }

    /**
     * Gets norm of noise root PSD (Power Spectral Density) among x,y,z
     * components expressed as (rad * s^-0.5).
     *
     * @return norm of noise root PSD.
     */
    public double getAngularRateNoiseRootPsdNorm() {
        return mAngularSpeedEstimator.getNoiseRootPsdNorm();
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return mAccelerationEstimator.getNumberOfProcessedSamples();
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
     * Adds body kinematics measurement samples.
     *
     * @param specificForceX x coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceY y coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceZ z coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param angularRateX   x coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateY   y coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateZ   z coordinate of angular rate expressed in radians per second (rad/s).
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final double specificForceX, final double specificForceY, final double specificForceZ,
            final double angularRateX, final double angularRateY, final double angularRateZ)
            throws LockedException {

        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;

        if (mLastBodyKinematics == null && mListener != null) {
            mListener.onStart(this);
        }

        if (mLastBodyKinematics == null) {
            mLastBodyKinematics = new BodyKinematics();
        }
        mLastBodyKinematics.setSpecificForceCoordinates(specificForceX, specificForceY, specificForceZ);
        mLastBodyKinematics.setAngularRateCoordinates(angularRateX, angularRateY, angularRateZ);

        mAccelerationEstimator.addTriad(specificForceX, specificForceY, specificForceZ);
        mAngularSpeedEstimator.addTriad(angularRateX, angularRateY, angularRateZ);

        if (mListener != null) {
            mListener.onBodyKinematicsAdded(this);
        }

        mRunning = false;
    }

    /**
     * Adds body kinematics measurement samples.
     *
     * @param specificForceX x coordinate of specific force.
     * @param specificForceY y coordinate of specific force.
     * @param specificForceZ z coordinate of specific force.
     * @param angularRateX   x coordinate of angular rate.
     * @param angularRateY   y coordinate of angular rate.
     * @param angularRateZ   z coordinate of angular rate.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final Acceleration specificForceX,
            final Acceleration specificForceY,
            final Acceleration specificForceZ,
            final AngularSpeed angularRateX,
            final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ)
            throws LockedException {
        addBodyKinematics(convertAcceleration(specificForceX),
                convertAcceleration(specificForceY),
                convertAcceleration(specificForceZ),
                convertAngularSpeed(angularRateX),
                convertAngularSpeed(angularRateY),
                convertAngularSpeed(angularRateZ));
    }

    /**
     * Adds body kinematics measurement samples.
     *
     * @param specificForce specific force triad.
     * @param angularSpeed  angular speed triad.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final AccelerationTriad specificForce,
            final AngularSpeedTriad angularSpeed) throws LockedException {
        addBodyKinematics(convertAcceleration(specificForce.getValueX(), specificForce.getUnit()),
                convertAcceleration(specificForce.getValueY(), specificForce.getUnit()),
                convertAcceleration(specificForce.getValueZ(), specificForce.getUnit()),
                convertAngularSpeed(angularSpeed.getValueX(), angularSpeed.getUnit()),
                convertAngularSpeed(angularSpeed.getValueY(), angularSpeed.getUnit()),
                convertAngularSpeed(angularSpeed.getValueZ(), angularSpeed.getUnit()));
    }

    /**
     * Adds body kinematics measurement.
     *
     * @param bodyKinematics body kinematics.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(final BodyKinematics bodyKinematics) throws LockedException {
        addBodyKinematics(bodyKinematics.getFx(),
                bodyKinematics.getFy(),
                bodyKinematics.getFz(),
                bodyKinematics.getAngularRateX(),
                bodyKinematics.getAngularRateY(),
                bodyKinematics.getAngularRateZ());
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

        mRunning = true;

        mLastBodyKinematics = null;
        final boolean result = mAccelerationEstimator.reset() && mAngularSpeedEstimator.reset();

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;

        return result;
    }

    /**
     * Converts an acceleration instance to meters per squared seconds (m/s^2).
     *
     * @param value value to be converted.
     * @return converted value.
     */
    private double convertAcceleration(final Acceleration value) {
        return AccelerationConverter.convert(
                value.getValue().doubleValue(),
                value.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts an acceleration value and unit to meters per squared seconds (m/s^2).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private double convertAcceleration(
            final double value, final AccelerationUnit unit) {
        return AccelerationConverter.convert(value, unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts an angular speed instance to radians per second (rad/s).
     *
     * @param value value to be converted.
     * @return converted value.
     */
    private double convertAngularSpeed(final AngularSpeed value) {
        return AngularSpeedConverter.convert(
                value.getValue().doubleValue(),
                value.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts an angular speed value and unit to radians per second (rad/s).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private double convertAngularSpeed(
            final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return getSpecificForceNoiseRootPsdNorm();
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return getAngularRateNoiseRootPsdNorm();
    }
}
