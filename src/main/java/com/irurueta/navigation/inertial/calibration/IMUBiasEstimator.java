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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Angle;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * This class estimates IMU accelerometer and gyroscope biases and noise PSD's by
 * averaging all provided samples when body position and orientation is known.
 * <p>
 * This estimator must be used when the body where the IMU is attached remains static
 * on the same position with zero velocity while capturing data.
 * <p>
 * To compute PSD's, this estimator assumes that IMU samples are obtained at a constant
 * provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, IMU sampling rate average can be estimated using
 * {@link IMUTimeIntervalEstimator}.
 * <p>
 * Notice that in order to compute accelerometer and gyroscope biases, body position
 * and orientation must be known to account for gravity and Earth rotation effects.
 * If only noise PSD's levels are required {@link IMUNoiseEstimator} can be used
 * instead, if body position and orientation is not available.
 */
public class IMUBiasEstimator {

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
     * Contains body position, velocity (which will always be zero) and orientation
     * resolved around ECEF axes.
     * By default it is assumed that body is located at zero NED coordinates (latitude,
     * longitude and height) and with zero Euler angles representing rotation (roll = 0,
     * pith = 0, yaw = 0), which for Android devices it means that the device is flat
     * on a horizontal surface with the screen facing down.
     */
    private ECEFFrame mFrame;

    /**
     * Listener to handle events raised by this estimator.
     */
    private IMUBiasEstimatorListener mListener;

    /**
     * Last provided body kinematics values.
     */
    private BodyKinematics mLastBodyKinematics;

    /**
     * Contains estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mBiasFx;

    /**
     * Contains estimated bias of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mBiasFy;

    /**
     * Contains estimated bias of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double mBiasFz;

    /**
     * Contains estimated bias of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mBiasAngularRateX;

    /**
     * Contains estimated bias of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mBiasAngularRateY;

    /**
     * Contains estimated bias of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double mBiasAngularRateZ;

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
     * force expressed in (m^2/s4).
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
     * Theoretical expected body kinematics for provided body position and orientation,
     * and provided time interval, assuming that body remains at the same position
     * (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     */
    private BodyKinematics mExpectedKinematics;

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     */
    public IMUBiasEstimator() {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC coordinate transformation from body to local navigation
     *             (NED) coordinates. This contains orientation respect the horizon
     *             at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height) {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height) {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height) {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(position, nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        mFrame = new ECEFFrame(position);
        final NEDFrame nedFrame = ECEFtoNEDFrameConverter
                .convertECEFtoNEDAndReturnNew(mFrame);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public IMUBiasEstimator(final IMUBiasEstimatorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param listener  listener to handle events raised by this estimator.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(final int totalSamples) {
        this();

        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(totalSamples);
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples) {
        this(totalSamples);
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples) {
        this(totalSamples);
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples) {
        this(totalSamples);
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(totalSamples);
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(position, nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);

        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mTotalSamples = totalSamples;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final int totalSamples, final IMUBiasEstimatorListener listener) {
        this(totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final double timeInterval) {
        this();
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final double latitude, final double longitude,
                            final double height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Angle latitude, final Angle longitude,
                            final double height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Angle latitude, final Angle longitude,
                            final Distance height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final double timeInterval,
                            final IMUBiasEstimatorListener listener) {
        this(timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final double timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(final int totalSamples, final double timeInterval) {
        this(totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples, final double timeInterval) {
        this(latitude, longitude, height, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples, final double timeInterval) {
        this(latitude, longitude, height, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples, final double timeInterval) {
        this(latitude, longitude, height, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(final int totalSamples, final double timeInterval,
                            final IMUBiasEstimatorListener listener) {
        this(totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples,
            final double timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples, final double timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples, final double timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples, final double timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final double timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final double timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Time timeInterval) {
        this(convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final double latitude, final double longitude,
                            final double height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Angle latitude, final Angle longitude,
                            final double height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Angle latitude, final Angle longitude,
                            final Distance height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(final Time timeInterval,
                            final IMUBiasEstimatorListener listener) {
        this(convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final Time timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Time timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final Time timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Time timeInterval, final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Time timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Time timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(final int totalSamples, final Time timeInterval) {
        this(totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples,
            final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples, final Time timeInterval) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided total samples is zero or negative, or
     *                                  if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples, final Time timeInterval) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples, final Time timeInterval) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative, or
     *                                                       if provided time interval
     *                                                       is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(final int totalSamples, final Time timeInterval,
                            final IMUBiasEstimatorListener listener) {
        this(totalSamples, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final CoordinateTransformation nedC, final int totalSamples,
            final Time timeInterval, final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, totalSamples, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final double latitude, final double longitude, final double height,
            final int totalSamples, final Time timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval),
                listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final int totalSamples, final Time timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval),
                listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative,
     *                                  or if provided time interval is negative.
     */
    public IMUBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final int totalSamples, final Time timeInterval,
            final IMUBiasEstimatorListener listener) {
        this(latitude, longitude, height, totalSamples, convertTime(timeInterval),
                listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final Time timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param totalSamples total samples to be processed to finish estimation.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided total samples
     *                                                       is zero or negative,
     *                                                       or if provided time
     *                                                       interval is negative.
     */
    public IMUBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final int totalSamples, final Time timeInterval,
            final IMUBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, totalSamples, convertTime(timeInterval), listener);
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

        rebuildExpectedKinematics();
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
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @return current body position expressed in ECEF coordinates.
     */
    public ECEFPosition getEcefPosition() {
        return mFrame.getECEFPosition();
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        mFrame.getECEFPosition(result);
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final ECEFPosition position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes and expressed in meters (m).
     * @param y y position resolved around ECEF axes and expressed in meters (m).
     * @param z z position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final double x, final double y, final double z)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes.
     * @param y y position resolved around ECEF axes.
     * @param z z position resolved around ECEF axes.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Distance x, final Distance y, final Distance z)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Point3D position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);
        rebuildExpectedKinematics();
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return ECEF frame containing current body position and orientation resolved
     * around ECEF axes.
     */
    public ECEFFrame getEcefFrame() {
        return new ECEFFrame(mFrame);
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where ECEF frame containing current body position and
     *               orientation resolved around ECEF axes will be stored.
     */
    public void getEcefFrame(final ECEFFrame result) {
        mFrame.copyTo(result);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return NED frame containing current body position and orientation resolved
     * around NED axes.
     */
    public NEDFrame getNedFrame() {
        return ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(mFrame);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where NED frame containing current body position and
     *               orientation resolved around NED axes will be stored.
     */
    public void getNedFrame(final NEDFrame result) {
        ECEFtoNEDFrameConverter.convertECEFtoNED(mFrame, result);
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @return current body position expressed in NED coordinates.
     */
    public NEDPosition getNedPosition() {
        return getNedFrame().getPosition();
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getNedPosition(final NEDPosition result) {
        getNedFrame().getPosition(result);
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(final NEDPosition position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate expressed in radians (rad).
     * @param longitude longitude NED coordinate expressed in radians (rad).
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final double latitude, final double longitude, final double height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final double height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final Distance height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @return current body orientation resolved on ECEF axes.
     */
    public CoordinateTransformation getEcefC() {
        return mFrame.getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on ECEF axes
     *               will be stored.
     */
    public void getEcefC(final CoordinateTransformation result) {
        mFrame.getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that ECEF orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizont at current body position.
     * Typically it is more convenient to use
     * {@link #setNedC(CoordinateTransformation)} to specify orientation relative to
     * the ground or horizon at current body position.
     * For instance, on Android devices a NED orientation with Euler angles (roll = 0,
     * pitch = 0, yaw = 0) means that the device is laying flat on a horizontal surface
     * with the screen facing down towards the ground.
     *
     * @param ecefC body orientation resolved on ECEF axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setEcefC(final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @return current body orientation resolved on NED axes.
     */
    public CoordinateTransformation getNedC() {
        return getNedFrame().getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on NED axes
     *               will be stored.
     */
    public void getNedC(final CoordinateTransformation result) {
        getNedFrame().getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to NED coordinates.
     * Notice that provided orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param nedC orientation resolved on NED axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setNedC(final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final NEDPosition nedPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(nedPosition);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param ecefC        body to ECEF coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(ecefPosition);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position expressed in meters (m).
     * @param y     y coordinate of ECEF position expressed in meters (m).
     * @param z     z coordinate of ECEF position expressed in meters (m).
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final double x, final double y, final double z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position.
     * @param y     y coordinate of ECEF position.
     * @param z     z coordinate of ECEF position.
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Distance x, final Distance y, final Distance z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Point3D position, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param position position expressed on NED coordinates.
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final NEDPosition position,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(ecefPosition);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position expressed in meters (m).
     * @param y    y coordinate of ECEF position expressed in meters (m).
     * @param z    z coordinate of ECEF position expressed in meters (m).
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final double x, final double y, final double z,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position.
     * @param y    y coordinate of ECEF position.
     * @param z    z coordinate of ECEF position.
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Distance x, final Distance y, final Distance z,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param nedC     body to NED coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Point3D position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        rebuildExpectedKinematics();
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public IMUBiasEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final IMUBiasEstimatorListener listener)
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
     * Gets estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of x coordinate of sensed specific force.
     */
    public double getBiasFx() {
        return mBiasFx;
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force.
     *
     * @return bias of x coordinate of sensed specific force.
     */
    public Acceleration getBiasFxAsAcceleration() {
        return new Acceleration(mBiasFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of x coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFxAsAcceleration(final Acceleration result) {
        result.setValue(mBiasFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of y coordinate of sensed specific force.
     */
    public double getBiasFy() {
        return mBiasFy;
    }

    /**
     * Gets estimated bias of y coordinate of accelerometer sensed specific force.
     *
     * @return bias of y coordinate of sensed specific force.
     */
    public Acceleration getBiasFyAsAcceleration() {
        return new Acceleration(mBiasFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of y coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFyAsAcceleration(final Acceleration result) {
        result.setValue(mBiasFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of z coordinate of sensed specific force.
     */
    public double getBiasFz() {
        return mBiasFz;
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force.
     *
     * @return bias of z coordinate of sensed specific force.
     */
    public Acceleration getBiasFzAsAcceleration() {
        return new Acceleration(mBiasFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of z coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFzAsAcceleration(final Acceleration result) {
        result.setValue(mBiasFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of x coordinate of sensed angular rate.
     */
    public double getBiasAngularRateX() {
        return mBiasAngularRateX;
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate.
     *
     * @return bias of x coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateXAsAngularSpeed() {
        return new AngularSpeed(mBiasAngularRateX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of x coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateXAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mBiasAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of y coordinate of sensed angular rate.
     */
    public double getBiasAngularRateY() {
        return mBiasAngularRateY;
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate.
     *
     * @return bias of y coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateYAsAngularSpeed() {
        return new AngularSpeed(mBiasAngularRateY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of y coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateYAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mBiasAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of z coordinate of sensed angular rate.
     */
    public double getBiasAngularRateZ() {
        return mBiasAngularRateZ;
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate.
     *
     * @return bias of z coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateZAsAngularSpeed() {
        return new AngularSpeed(mBiasAngularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of z coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateZAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mBiasAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets body kinematics containing estimated bias values for accelerometer
     * and gyroscope.
     *
     * @return body kinematics containing estimated bias values.
     */
    public BodyKinematics getBiasesAsBodyKinematics() {
        final BodyKinematics result = new BodyKinematics();
        getBiasesAsBodyKinematics(result);
        return result;
    }

    /**
     * Gets body kinematics containing estimated bias values for accelerometer
     * and gyroscope.
     *
     * @param result instance where body kinematics containing estimated bias
     *               values will be stored.
     */
    public void getBiasesAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(mBiasFx, mBiasFy, mBiasFz);
        result.setAngularRateCoordinates(
                mBiasAngularRateX, mBiasAngularRateY, mBiasAngularRateZ);
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
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates expressed in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation of accelerometer.
     */
    public double getAverageAccelerometerStandardDeviation() {
        return (getStandardDeviationFx() + getStandardDeviationFy()
                + getStandardDeviationFz()) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates.
     *
     * @return average of estimated standard deviation of accelerometer.
     */
    public Acceleration getAverageAccelerometerStandardDeviationAsAcceleration() {
        return new Acceleration(getAverageAccelerometerStandardDeviation(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates.
     *
     * @param result instance where result data will be copied to.
     */
    public void getAverageAccelerometerStandardDeviationAsAcceleration(
            final Acceleration result) {
        result.setValue(getAverageAccelerometerStandardDeviation());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviaton of x coordinate of sensed angular rate.
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
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates expressed in radians per second (rad/s).
     *
     * @return average of estimated standard deviation of gyroscope.
     */
    public double getAverageGyroscopeStandardDeviation() {
        return (getStandardDeviationAngularRateX() + getStandardDeviationAngularRateY()
                + getStandardDeviationAngularRateZ()) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates.
     *
     * @return average of estimated standard deviation of gyroscope.
     */
    public AngularSpeed getAverageGyroscopeStandardDeviationAsAngularSpeed() {
        return new AngularSpeed(getAverageGyroscopeStandardDeviation(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates.
     *
     * @param result instance where result data will be copied to.
     */
    public void getAverageGyroscopeStandardDeviationAsAngularSpeed(
            final AngularSpeed result) {
        result.setValue(getAverageGyroscopeStandardDeviation());
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
     * x,y,z components expressed as (m^2/s^3).
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
     * Gets estimated bias of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2) as a 3x1 matrix column vector.
     *
     * @return estimated bias of accelerometer sensed specific force.
     */
    public Matrix getAccelerometerBias() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getAccelerometerBias(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }

        return result;
    }

    /**
     * Gets estimated bias of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2) as a 3x1 matrix column vector.
     *
     * @param result instance where data will be copied to. Must be 3x1.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getAccelerometerBias(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        result.setElementAtIndex(0, mBiasFx);
        result.setElementAtIndex(1, mBiasFy);
        result.setElementAtIndex(2, mBiasFz);
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rates
     * expressed in radians per second (rad/s) as a 3x1 matrix column vector.
     *
     * @return estimated bias of gyroscope sensed angular rates.
     */
    public Matrix getGyroBias() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getGyroBias(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }

        return result;
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rates
     * expressed in radians per second (rad/s) as a 3x1 matrix column vector.
     *
     * @param result instance where data will be copied to. Must be 3x1.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getGyroBias(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        result.setElementAtIndex(0, mBiasAngularRateX);
        result.setElementAtIndex(1, mBiasAngularRateY);
        result.setElementAtIndex(2, mBiasAngularRateZ);
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
     * Gets theoretically expected body kinematics for provided body position and
     * orientation, and provided time interval, assuming that body remains at the
     * same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     *
     * @return expected body kinematics.
     */
    public BodyKinematics getExpectedKinematics() {
        return new BodyKinematics(mExpectedKinematics);
    }

    /**
     * Gets theoretically expected body kinematics for provided body position and
     * orientation, and provided time interval, assuming that body remains at the
     * same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     *
     * @param result instance where expected body kinematics will be stored.
     */
    public void getExpectedKinematics(final BodyKinematics result) {
        mExpectedKinematics.copyTo(result);
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

        final double expectedFx = mExpectedKinematics.getFx();
        final double expectedFy = mExpectedKinematics.getFy();
        final double expectedFz = mExpectedKinematics.getFz();
        final double expectedAngularRateX = mExpectedKinematics.getAngularRateX();
        final double expectedAngularRateY = mExpectedKinematics.getAngularRateY();
        final double expectedAngularRateZ = mExpectedKinematics.getAngularRateZ();

        final double diffFx = fx - expectedFx;
        final double diffFy = fy - expectedFy;
        final double diffFz = fz - expectedFz;
        final double diffAngularRateX = angularRateX - expectedAngularRateX;
        final double diffAngularRateY = angularRateY - expectedAngularRateY;
        final double diffAngularRateZ = angularRateZ - expectedAngularRateZ;

        // compute biases
        mBiasFx = mBiasFx * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFx / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasFy = mBiasFy * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFy / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasFz = mBiasFz * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffFz / (double) mNumberOfProcessedSamplesPlusOne;

        mBiasAngularRateX = mBiasAngularRateX * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateX / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasAngularRateY = mBiasAngularRateY * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateY / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasAngularRateZ = mBiasAngularRateZ * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffAngularRateZ / (double) mNumberOfProcessedSamplesPlusOne;

        // compute variances
        final double diffBiasFx = diffFx - mBiasFx;
        final double diffBiasFy = diffFy - mBiasFy;
        final double diffBiasFz = diffFz - mBiasFz;
        final double diffBiasAngularRateX = diffAngularRateX - mBiasAngularRateX;
        final double diffBiasAngularRateY = diffAngularRateY - mBiasAngularRateY;
        final double diffBiasAngularRateZ = diffAngularRateZ - mBiasAngularRateZ;

        final double diffBiasFx2 = diffBiasFx * diffBiasFx;
        final double diffBiasFy2 = diffBiasFy * diffBiasFy;
        final double diffBiasFz2 = diffBiasFz * diffBiasFz;
        final double diffBiasAngularRateX2 =
                diffBiasAngularRateX * diffBiasAngularRateX;
        final double diffBiasAngularRateY2 =
                diffBiasAngularRateY * diffBiasAngularRateY;
        final double diffBiasAngularRateZ2 =
                diffBiasAngularRateZ * diffBiasAngularRateZ;

        mVarianceFx = mVarianceFx * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasFx2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceFy = mVarianceFy * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasFy2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceFz = mVarianceFz * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasFz2 / (double) mNumberOfProcessedSamplesPlusOne;

        mVarianceAngularRateX = mVarianceAngularRateX
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasAngularRateX2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceAngularRateY = mVarianceAngularRateY
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasAngularRateY2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceAngularRateZ = mVarianceAngularRateZ
                * (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne
                + diffBiasAngularRateZ2 / (double) mNumberOfProcessedSamplesPlusOne;

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
        mBiasFx = 0.0;
        mBiasFy = 0.0;
        mBiasFz = 0.0;
        mBiasAngularRateX = 0.0;
        mBiasAngularRateY = 0.0;
        mBiasAngularRateZ = 0.0;
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

    /**
     * Converts provided time instance to seconds.
     *
     * @param time instance to be converted.
     * @return obtained conversion in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Rebuilds expected theoretical kinematics for provided body position
     * and orientation and provided time interval, assuming that body
     * remains at the same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     */
    private void rebuildExpectedKinematics() {
        if (mFrame == null) {
            return;
        }
        if (mExpectedKinematics == null) {
            mExpectedKinematics = new BodyKinematics();
        }

        final CoordinateTransformation ecefC = getEcefC();
        final double x = mFrame.getX();
        final double y = mFrame.getY();
        final double z = mFrame.getZ();
        ECEFKinematicsEstimator.estimateKinematics(mTimeInterval, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                x, y, z, mExpectedKinematics);
    }
}
