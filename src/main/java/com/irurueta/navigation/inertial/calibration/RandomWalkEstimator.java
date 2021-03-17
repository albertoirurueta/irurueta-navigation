/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.bias.BodyKinematicsBiasEstimator;
import com.irurueta.units.*;

/**
 * Estimates random walk data by running this estimator while the body of IMU remains static
 * at a given position and orientation when IMU has already been calibrated.
 * Internally this estimator accumulates samples for a given "drift period" to
 * estimate accumulated drift values of position, velocity and attitude, and then
 * repeats the process several times to estimate mean and variance of such values.
 * The duration of the drift period is application dependant and should be set
 * to a value more or less similar to the amount of time the device won't be able
 * to set a position by some other system (e.g. GPS, Wi-Fi, cameras, etc...)
 */
public class RandomWalkEstimator implements AccelerometerBiasRandomWalkSource,
        GyroscopeBiasRandomWalkSource, PositionUncertaintySource,
        VelocityUncertaintySource, AttitudeUncertaintySource,
        PositionNoiseStandardDeviationSource, VelocityNoiseStandardDeviationSource {

    /**
     * Number of samples to be used by default on each drift period.
     */
    public static final int DEFAULT_DRIFT_PERIOD_SAMPLES = 150;

    /**
     * Listener to handle events raised by this estimator.
     */
    private RandomWalkEstimatorListener mListener;

    /**
     * Fixes body kinematics measurements using accelerometer + gyroscope
     * calibration data to fix measurements.
     */
    private final BodyKinematicsFixer mFixer = new BodyKinematicsFixer();

    /**
     * Estimates bias for fixed body kinematics measurements to determine furhter
     * bias variations while the IMU body remains static.
     */
    private final BodyKinematicsBiasEstimator mBiasEstimator = new BodyKinematicsBiasEstimator();

    /**
     * Estimates amount of position, velocity and orientation drift.
     */
    private final DriftEstimator mDriftEstimator = new DriftEstimator();

    /**
     * Instance containing last fixed body kinematics to be reused.
     */
    private final BodyKinematics mFixedKinematics = new BodyKinematics();

    /**
     * Indicates whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used, otherwise it is
     * ignored.
     * By default this is enabled.
     */
    private boolean mFixKinematics = true;

    /**
     * Number of samples to be used on each drift period.
     */
    private int mDriftPeriodSamples = DEFAULT_DRIFT_PERIOD_SAMPLES;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean mRunning;

    /**
     * Number of drift periods that have been processed.
     */
    private int mNumberOfProcessedDriftPeriods;

    /**
     * Accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     */
    private double mAccelerometerBiasPSD;

    /**
     * Gyro bias random walk PSD (Power Spectral Density) expressed in (rad^2 * s^-3).
     */
    private double mGyroBiasPSD;

    /**
     * Average position drift expressed in meters (m).
     * This gives an indication of position accuracy.
     */
    private double mAvgPositionDrift;

    /**
     * Average velocity drift expressed in meters per second (m/s).
     */
    private double mAvgVelocityDrift;

    /**
     * Average attitude drift expressed in radians (rad).
     */
    private double mAvgAttitudeDrift;

    /**
     * Position variance expressed in squared meters (m^2).
     */
    private double mVarPositionDrift;

    /**
     * Velocity variance expressed in (m^2/s^2).
     */
    private double mVarVelocityDrift;

    /**
     * Attitude variance expressed in squared radians (rad^2).
     */
    private double mVarAttitudeDrift;

    /**
     * Contains acceleration triad to be reused for bias norm estimation.
     * This is reused for efficiency.
     */
    private final AccelerationTriad mAccelerationTriad = new AccelerationTriad();

    /**
     * Contains angular speed triad to be reused for bias norm estimation.
     * This is reused for efficiency.
     */
    private final AngularSpeedTriad mAngularSpeedTriad = new AngularSpeedTriad();

    /**
     * Constructor.
     */
    public RandomWalkEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events.
     */
    public RandomWalkEstimator(final RandomWalkEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set.
     * @param ma acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set.
     * @param mg angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg) throws AlgebraException {
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set.
     * @param ma       acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set.
     * @param mg       angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener)
            throws AlgebraException {
        this(ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set.
     * @param ma acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set.
     * @param mg angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set.
     * @param ma       acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set.
     * @param mg       angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg       angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set expressed in meters per squared second
     *           (m/s`2). Must be 3x1.
     * @param ma acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set expressed in radians per second
     *           (rad/s). Must be 3x1.
     * @param mg angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices do not have proper
     *                                  size.
     */
    public RandomWalkEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set expressed in meters per squared second
     *                 (m/s`2). Must be 3x1.
     * @param ma       acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set expressed in radians per second
     *                 (rad/s). Must be 3x1.
     * @param mg       angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices do not have proper
     *                                  size.
     */
    public RandomWalkEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set expressed in meters per squared second
     *           (m/s`2). Must be 3x1.
     * @param ma acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set expressed in radians per second
     *           (rad/s). Must be 3x1.
     * @param mg angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices do not have proper
     *                                  size.
     */
    public RandomWalkEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set expressed in meters per squared second
     *                 (m/s`2). Must be 3x1.
     * @param ma       acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set expressed in radians per second
     *                 (rad/s). Must be 3x1.
     * @param mg       angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg       angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices do not have proper
     *                                  size.
     */
    public RandomWalkEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        try {
            setNedPositionAndNedOrientation(nedPosition, nedC);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param listener    listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedPosition, nedC);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener    listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg          angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg          angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener    listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set expressed in meters per squared second
     *                    (m/s`2). Must be 3x1.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set expressed in radians per second
     *                    (rad/s). Must be 3x1.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set expressed in meters per squared second
     *                    (m/s`2). Must be 3x1.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set expressed in radians per second
     *                    (rad/s). Must be 3x1.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener    listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set expressed in meters per squared second
     *                    (m/s`2). Must be 3x1.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set expressed in radians per second
     *                    (rad/s). Must be 3x1.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg          angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @param ba          acceleration bias to be set expressed in meters per squared second
     *                    (m/s`2). Must be 3x1.
     * @param ma          acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg          angular speed bias to be set expressed in radians per second
     *                    (rad/s). Must be 3x1.
     * @param mg          angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg          angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener    listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        try {
            setEcefPositionAndNedOrientation(ecefPosition, nedC);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(ecefPosition, nedC);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC);
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param nedPosition  position expressed on NED coordinates.
     * @param nedC         body to NED coordinate transformation indicating
     *                     body orientation.
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final NEDPosition nedPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, timeInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param timeInterval time interval between body kinematics samples expressed
     *                     in seconds (s).
     * @param listener     listener to handle events.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws AlgebraException                              if provided cross coupling matrices cannot
     *                                                       be inverted.
     * @throws IllegalArgumentException                      if provided matrices are not 3x3.
     */
    public RandomWalkEstimator(
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final double timeInterval,
            final RandomWalkEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, AlgebraException {
        this(ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval);
        mListener = listener;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RandomWalkEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if estimator is running.
     */
    public void setListener(final RandomWalkEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public Matrix getAccelerationBias() {
        return mFixer.getAccelerationBias();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBias(final Matrix result) {
        mFixer.getAccelerationBias(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second.
     *             Must be 3x1.
     * @throws LockedException          if estimator is running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setAccelerationBias(final Matrix bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBias(bias);
        mDriftEstimator.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public double[] getAccelerationBiasArray() {
        return mFixer.getAccelerationBiasArray();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAccelerationBiasArray(final double[] result) {
        mFixer.getAccelerationBiasArray(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second (m/s^2).
     *             Must have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     * @throws LockedException          if estimator is running.
     */
    public void setAccelerationBias(final double[] bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBias(bias);
        mDriftEstimator.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration bias.
     *
     * @return acceleration bias.
     */
    public AccelerationTriad getAccelerationBiasAsTriad() {
        return mFixer.getAccelerationBiasAsTriad();
    }

    /**
     * Gets acceleration bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasAsTriad(final AccelerationTriad result) {
        mFixer.getAccelerationBiasAsTriad(result);
    }

    /**
     * Sets acceleration bias.
     *
     * @param bias acceleration bias to be set.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(final AccelerationTriad bias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBias(bias);
        mDriftEstimator.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasX() {
        return mFixer.getAccelerationBiasX();
    }

    /**
     * Sets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasX(biasX);
        mDriftEstimator.setAccelerationBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasY() {
        return mFixer.getAccelerationBiasY();
    }

    /**
     * Sets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasY(final double biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasY(biasY);
        mDriftEstimator.setAccelerationBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasZ() {
        return mFixer.getAccelerationBiasZ();
    }

    /**
     * Sets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasZ(final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasZ(biasZ);
        mDriftEstimator.setAccelerationBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(
            final double biasX, final double biasY, final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBias(biasX, biasY, biasZ);
        mDriftEstimator.setAccelerationBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @return acceleration x-coordinate of bias.
     */
    public Acceleration getAccelerationBiasXAsAcceleration() {
        return mFixer.getAccelerationBiasXAsAcceleration();
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasXAsAcceleration(final Acceleration result) {
        mFixer.getAccelerationBiasXAsAcceleration(result);
    }

    /**
     * Sets acceleration x-coordinate of bias.
     *
     * @param biasX acceleration x-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasX(final Acceleration biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasX(biasX);
        mDriftEstimator.setAccelerationBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @return acceleration y-coordinate of bias.
     */
    public Acceleration getAccelerationBiasYAsAcceleration() {
        return mFixer.getAccelerationBiasYAsAcceleration();
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasYAsAcceleration(final Acceleration result) {
        mFixer.getAccelerationBiasYAsAcceleration(result);
    }

    /**
     * Sets acceleration y-coordinate of bias.
     *
     * @param biasY acceleration y-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasY(final Acceleration biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasY(biasY);
        mDriftEstimator.setAccelerationBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @return acceleration z-coordinate of bias.
     */
    public Acceleration getAccelerationBiasZAsAcceleration() {
        return mFixer.getAccelerationBiasZAsAcceleration();
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasZAsAcceleration(final Acceleration result) {
        mFixer.getAccelerationBiasZAsAcceleration(result);
    }

    /**
     * Sets acceleration z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasZ(final Acceleration biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasZ(biasZ);
        mDriftEstimator.setAccelerationBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(
            final Acceleration biasX,
            final Acceleration biasY,
            final Acceleration biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBias(biasX, biasY, biasZ);
        mDriftEstimator.setAccelerationBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration cross coupling errors matrix.
     *
     * @return acceleration cross coupling errors matrix.
     */
    public Matrix getAccelerationCrossCouplingErrors() {
        return mFixer.getAccelerationCrossCouplingErrors();
    }

    /**
     * Gets acceleration cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationCrossCouplingErrors(final Matrix result) {
        mFixer.getAccelerationCrossCouplingErrors(result);
    }

    /**
     * Sets acceleration cross coupling errors matrix.
     *
     * @param crossCouplingErrors acceleration cross coupling errors matrix.
     *                            Must be 3x3.
     * @throws LockedException          if estimator is running.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAccelerationCrossCouplingErrors(
            final Matrix crossCouplingErrors) throws AlgebraException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationCrossCouplingErrors(crossCouplingErrors);
        mDriftEstimator.setAccelerationCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets acceleration x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAccelerationSx() {
        return mFixer.getAccelerationSx();
    }

    /**
     * Sets acceleration x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationSx(final double sx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationSx(sx);
        mDriftEstimator.setAccelerationSx(sx);
    }

    /**
     * Gets acceleration y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAccelerationSy() {
        return mFixer.getAccelerationSy();
    }

    /**
     * Sets acceleration y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationSy(final double sy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationSy(sy);
        mDriftEstimator.setAccelerationSy(sy);
    }

    /**
     * Gets acceleration z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAccelerationSz() {
        return mFixer.getAccelerationSz();
    }

    /**
     * Sets acceleration z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationSz(final double sz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationSz(sz);
        mDriftEstimator.setAccelerationSz(sz);
    }

    /**
     * Gets acceleration x-y cross coupling error.
     *
     * @return acceleration x-y cross coupling error.
     */
    public double getAccelerationMxy() {
        return mFixer.getAccelerationMxy();
    }

    /**
     * Sets acceleration x-y cross coupling error.
     *
     * @param mxy acceleration x-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMxy(final double mxy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMxy(mxy);
        mDriftEstimator.setAccelerationMxy(mxy);
    }

    /**
     * Gets acceleration x-z cross coupling error.
     *
     * @return acceleration x-z cross coupling error.
     */
    public double getAccelerationMxz() {
        return mFixer.getAccelerationMxz();
    }

    /**
     * Sets acceleration x-z cross coupling error.
     *
     * @param mxz acceleration x-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMxz(final double mxz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMxz(mxz);
        mDriftEstimator.setAccelerationMxz(mxz);
    }

    /**
     * Gets acceleration y-x cross coupling error.
     *
     * @return acceleration y-x cross coupling error.
     */
    public double getAccelerationMyx() {
        return mFixer.getAccelerationMyx();
    }

    /**
     * Sets acceleration y-x cross coupling error.
     *
     * @param myx acceleration y-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMyx(final double myx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMyx(myx);
        mDriftEstimator.setAccelerationMyx(myx);
    }

    /**
     * Gets acceleration y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAccelerationMyz() {
        return mFixer.getAccelerationMyz();
    }

    /**
     * Sets acceleration y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMyz(final double myz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMyz(myz);
        mDriftEstimator.setAccelerationMyz(myz);
    }

    /**
     * Gets acceleration z-x cross coupling error.
     *
     * @return acceleration z-x cross coupling error.
     */
    public double getAccelerationMzx() {
        return mFixer.getAccelerationMzx();
    }

    /**
     * Sets acceleration z-x cross coupling error.
     *
     * @param mzx acceleration z-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMzx(final double mzx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMzx(mzx);
        mDriftEstimator.setAccelerationMzx(mzx);
    }

    /**
     * Gets acceleration z-y cross coupling error.
     *
     * @return acceleration z-y cross coupling error.
     */
    public double getAccelerationMzy() {
        return mFixer.getAccelerationMzy();
    }

    /**
     * Sets acceleration z-y cross coupling error.
     *
     * @param mzy acceleration z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationMzy(final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationMzy(mzy);
        mDriftEstimator.setAccelerationMzy(mzy);
    }

    /**
     * Sets acceleration scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationScalingFactors(
            final double sx, final double sy, final double sz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationScalingFactors(sx, sy, sz);
        mDriftEstimator.setAccelerationScalingFactors(sx, sy, sz);
    }

    /**
     * Sets acceleration cross coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);
        mDriftEstimator.setAccelerationCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets acceleration scaling factors and cross coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross
     *                          coupling matrix non invertible.
     */
    public void setAccelerationScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
        mDriftEstimator.setAccelerationScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return angular speed bias values expressed in radians per second.
     */
    public Matrix getAngularSpeedBias() {
        return mFixer.getAngularSpeedBias();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBias(final Matrix result) {
        mFixer.getAngularSpeedBias(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second. Must be 3x1.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedBias(final Matrix bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBias(bias);
        mDriftEstimator.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return bias values expressed in radians per second.
     */
    public double[] getAngularSpeedBiasArray() {
        return mFixer.getAngularSpeedBiasArray();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAngularSpeedBiasArray(final double[] result) {
        mFixer.getAngularSpeedBiasArray(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second (rad/s). Must
     *             have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedBias(final double[] bias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBias(bias);
        mDriftEstimator.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed bias.
     *
     * @return angular speed bias.
     */
    public AngularSpeedTriad getAngularSpeedBiasAsTriad() {
        return mFixer.getAngularSpeedBiasAsTriad();
    }

    /**
     * Gets angular speed bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasAsTriad(final AngularSpeedTriad result) {
        mFixer.getAngularSpeedBiasAsTriad(result);
    }

    /**
     * Sets angular speed bias.
     *
     * @param bias angular speed bias to be set.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(final AngularSpeedTriad bias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBias(bias);
        mDriftEstimator.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return x-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasX() {
        return mFixer.getAngularSpeedBiasX();
    }

    /**
     * Sets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasX(final double biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasX(biasX);
        mDriftEstimator.setAngularSpeedBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return y-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasY() {
        return mFixer.getAngularSpeedBiasY();
    }

    /**
     * Sets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasY(final double biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasY(biasY);
        mDriftEstimator.setAngularSpeedBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return z-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasZ() {
        return mFixer.getAngularSpeedBiasZ();
    }

    /**
     * Sets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasZ(final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasZ(biasZ);
        mDriftEstimator.setAngularSpeedBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(
            final double biasX, final double biasY, final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBias(biasX, biasY, biasZ);
        mDriftEstimator.setAngularSpeedBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @return x-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasXAsAngularSpeed() {
        return mFixer.getAngularSpeedBiasXAsAngularSpeed();
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasXAsAngularSpeed(
            final AngularSpeed result) {
        mFixer.getAngularSpeedBiasXAsAngularSpeed(result);
    }

    /**
     * Sets angular speed x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasX(final AngularSpeed biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasX(biasX);
        mDriftEstimator.setAngularSpeedBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasYAsAngularSpeed() {
        return mFixer.getAngularSpeedBiasYAsAngularSpeed();
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasYAsAngularSpeed(final AngularSpeed result) {
        mFixer.getAngularSpeedBiasYAsAngularSpeed(result);
    }

    /**
     * Sets angular speed y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasY(final AngularSpeed biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasY(biasY);
        mDriftEstimator.setAngularSpeedBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasZAsAngularSpeed() {
        return mFixer.getAngularSpeedBiasZAsAngularSpeed();
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasZAsAngularSpeed(final AngularSpeed result) {
        mFixer.getAngularSpeedBiasZAsAngularSpeed(result);
    }

    /**
     * Sets angular speed z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasZ(final AngularSpeed biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBiasZ(biasZ);
        mDriftEstimator.setAngularSpeedBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(
            final AngularSpeed biasX,
            final AngularSpeed biasY,
            final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedBias(biasX, biasY, biasZ);
        mDriftEstimator.setAngularSpeedBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed cross coupling errors matrix.
     *
     * @return cross coupling errors matrix.
     */
    public Matrix getAngularSpeedCrossCouplingErrors() {
        return mFixer.getAngularSpeedCrossCouplingErrors();
    }

    /**
     * Gets angular speed cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedCrossCouplingErrors(final Matrix result) {
        mFixer.getAngularSpeedCrossCouplingErrors(result);
    }

    /**
     * Sets angular speed cross coupling errors matrix.
     *
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedCrossCouplingErrors(
            final Matrix crossCouplingErrors) throws AlgebraException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedCrossCouplingErrors(crossCouplingErrors);
        mDriftEstimator.setAngularSpeedCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets angular speed x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAngularSpeedSx() {
        return mFixer.getAngularSpeedSx();
    }

    /**
     * Sets angular speed x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedSx(final double sx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedSx(sx);
        mDriftEstimator.setAngularSpeedSx(sx);
    }

    /**
     * Gets angular speed y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAngularSpeedSy() {
        return mFixer.getAngularSpeedSy();
    }

    /**
     * Sets angular speed y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedSy(final double sy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedSy(sy);
        mDriftEstimator.setAngularSpeedSy(sy);
    }

    /**
     * Gets angular speed z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAngularSpeedSz() {
        return mFixer.getAngularSpeedSz();
    }

    /**
     * Sets angular speed z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedSz(final double sz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedSz(sz);
        mDriftEstimator.setAngularSpeedSz(sz);
    }

    /**
     * Gets angular speed x-y cross coupling error.
     *
     * @return x-y cross coupling error.
     */
    public double getAngularSpeedMxy() {
        return mFixer.getAngularSpeedMxy();
    }

    /**
     * Sets angular speed x-y cross coupling error.
     *
     * @param mxy x-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMxy(final double mxy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMxy(mxy);
        mDriftEstimator.setAngularSpeedMxy(mxy);
    }

    /**
     * Gets angular speed x-z cross coupling error.
     *
     * @return x-z cross coupling error.
     */
    public double getAngularSpeedMxz() {
        return mFixer.getAngularSpeedMxz();
    }

    /**
     * Sets angular speed x-z cross coupling error.
     *
     * @param mxz x-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMxz(final double mxz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMxz(mxz);
        mDriftEstimator.setAngularSpeedMxz(mxz);
    }

    /**
     * Gets angular speed y-x cross coupling error.
     *
     * @return y-x cross coupling error.
     */
    public double getAngularSpeedMyx() {
        return mFixer.getAngularSpeedMyx();
    }

    /**
     * Sets angular speed y-x cross coupling error.
     *
     * @param myx y-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMyx(final double myx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMyx(myx);
        mDriftEstimator.setAngularSpeedMyx(myx);
    }

    /**
     * Gets angular speed y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAngularSpeedMyz() {
        return mFixer.getAngularSpeedMyz();
    }

    /**
     * Sets angular speed y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMyz(final double myz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMyz(myz);
        mDriftEstimator.setAngularSpeedMyz(myz);
    }

    /**
     * Gets angular speed z-x cross coupling error.
     *
     * @return z-x cross coupling error.
     */
    public double getAngularSpeedMzx() {
        return mFixer.getAngularSpeedMzx();
    }

    /**
     * Sets angular speed z-x cross coupling error.
     *
     * @param mzx z-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMzx(final double mzx)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMzx(mzx);
        mDriftEstimator.setAngularSpeedMzx(mzx);
    }

    /**
     * Gets angular speed z-y cross coupling error.
     *
     * @return z-y cross coupling error.
     */
    public double getAngularSpeedMzy() {
        return mFixer.getAngularSpeedMzy();
    }

    /**
     * Sets angular speed z-y cross coupling error.
     *
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedMzy(final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedMzy(mzy);
        mDriftEstimator.setAngularSpeedMzy(mzy);
    }

    /**
     * Sets angular speed scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedScalingFactors(
            final double sx, final double sy, final double sz)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedScalingFactors(sx, sy, sz);
        mDriftEstimator.setAngularSpeedScalingFactors(sx, sy, sz);
    }

    /**
     * Sets angular speed cross coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);
        mDriftEstimator.setAngularSpeedCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets angular speed scaling factors and cross coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross coupling matrix non invertible.
     */
    public void setAngularSpeedScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy)
            throws LockedException, AlgebraException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
        mDriftEstimator.setAngularSpeedScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @return g-dependant cross biases matrix.
     */
    public Matrix getAngularSpeedGDependantCrossBias() {
        return mFixer.getAngularSpeedGDependantCrossBias();
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedGDependantCrossBias(final Matrix result) {
        mFixer.getAngularSpeedGDependantCrossBias(result);
    }

    /**
     * Sets angular speed g-dependant cross biases matrix.
     *
     * @param gDependantCrossBias g-dependant cross biases matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedGDependantCrossBias(final Matrix gDependantCrossBias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAngularSpeedGDependantCrossBias(gDependantCrossBias);
        mDriftEstimator.setAngularSpeedGDependantCrossBias(gDependantCrossBias);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return mBiasEstimator.getTimeInterval();
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setTimeInterval(timeInterval);
        mDriftEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return mBiasEstimator.getTimeIntervalAsTime();
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        mBiasEstimator.getTimeIntervalAsTime(result);
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setTimeInterval(timeInterval);
        mDriftEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @return current body position expressed in ECEF coordinates.
     */
    public ECEFPosition getEcefPosition() {
        return mBiasEstimator.getEcefPosition();
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        mBiasEstimator.getEcefPosition(result);
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final ECEFPosition position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPosition(position);
        mDriftEstimator.setReferenceEcefPosition(position);
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

        mBiasEstimator.setEcefPosition(x, y, z);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
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

        mBiasEstimator.setEcefPosition(x, y, z);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Point3D position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPosition(position);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
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
        return mBiasEstimator.getEcefFrame();
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
        mBiasEstimator.getEcefFrame(result);
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
        return mBiasEstimator.getNedFrame();
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
        mBiasEstimator.getNedFrame(result);
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @return current body position expressed in NED coordinates.
     */
    public NEDPosition getNedPosition() {
        return mBiasEstimator.getNedPosition();
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getNedPosition(final NEDPosition result) {
        mBiasEstimator.getNedPosition(result);
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(final NEDPosition position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPosition(position);
        mDriftEstimator.setReferenceNedPosition(position);
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

        mBiasEstimator.setNedPosition(latitude, longitude, height);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
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

        mBiasEstimator.setNedPosition(latitude, longitude, height);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
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

        mBiasEstimator.setNedPosition(latitude, longitude, height);
        mDriftEstimator.setReferenceEcefPosition(
                mBiasEstimator.getEcefPosition());
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
        return mBiasEstimator.getEcefC();
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
        mBiasEstimator.getEcefC(result);
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

        mBiasEstimator.setEcefC(ecefC);
        mDriftEstimator.setReferenceEcefCoordinateTransformation(ecefC);
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
        return mBiasEstimator.getNedC();
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
        mBiasEstimator.getNedC(result);
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedC(nedC);
        mDriftEstimator.setReferenceNedCoordinateTransformation(nedC);
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndNedOrientation(nedPosition, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndNedOrientation(latitude, longitude,
                height, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndNedOrientation(latitude, longitude,
                height, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndNedOrientation(latitude, longitude,
                height, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            final ECEFPosition ecefPosition,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndEcefOrientation(ecefPosition, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndEcefOrientation(position, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndEcefOrientation(position, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndEcefOrientation(latitude, longitude,
                height, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndEcefOrientation(latitude, longitude,
                height, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setNedPositionAndEcefOrientation(latitude, longitude,
                height, ecefC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            final ECEFPosition ecefPosition,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndNedOrientation(ecefPosition, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndNedOrientation(x, y, z, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndNedOrientation(x, y, z, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
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
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasEstimator.setEcefPositionAndNedOrientation(position, nedC);
        mDriftEstimator.setReferenceFrame(mBiasEstimator.getEcefFrame());
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return mBiasEstimator.getNumberOfProcessedSamples();
    }

    /**
     * Gets number of drift periods that have been processed.
     *
     * @return number of drift periods that have been processed.
     */
    public int getNumberOfProcessedDriftPeriods() {
        return mNumberOfProcessedDriftPeriods;
    }

    /**
     * Gets amount of total elapsed time since first processed measurement expressed
     * in seconds (s).
     *
     * @return amount of total elapsed time.
     */
    public double getElapsedTimeSeconds() {
        return mBiasEstimator.getElapsedTimeSeconds();
    }

    /**
     * Gets amount of total elapsed time since first processed measurement.
     *
     * @return amount of total elapsed time.
     */
    public Time getElapsedTime() {
        return mBiasEstimator.getElapsedTime();
    }

    /**
     * Gets amount of total elapsed time since first processed measurement.
     *
     * @param result instance where result will be stored.
     */
    public void getElapsedTime(final Time result) {
        mBiasEstimator.getElapsedTime(result);
    }

    /**
     * Indicates whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used, otherwise it is
     * ignored.
     * By default this is enabled.
     *
     * @return indicates whether measured kinematics must be fixed or not.
     */
    public boolean isFixKinematicsEnabled() {
        return mFixKinematics;
    }

    /**
     * Specifies whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used, otherwise it is
     * ignored.
     *
     * @param fixKinematics true if measured kinematics must be fixed or not.
     * @throws LockedException if estimator is currently running.
     */
    public void setFixKinematicsEnabled(final boolean fixKinematics)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mFixKinematics = fixKinematics;
        mDriftEstimator.setFixKinematicsEnabled(fixKinematics);
    }

    /**
     * Gets number of samples to be used on each drift period.
     *
     * @return number of samples to be used on each drift period.
     */
    public int getDriftPeriodSamples() {
        return mDriftPeriodSamples;
    }

    /**
     * Sets number of samples to be used on each drift period.
     *
     * @param driftPeriodSamples number of samples to be used on each drift period.
     * @throws LockedException if estimator is currently running.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setDriftPeriodSamples(int driftPeriodSamples)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (driftPeriodSamples <= 0) {
            throw new IllegalArgumentException();
        }

        mDriftPeriodSamples = driftPeriodSamples;
    }

    /**
     * Gets duration of each drift period expressed in seconds (s).
     *
     * @return duration of each drift period.
     */
    public double getDriftPeriodSeconds() {
        return mDriftPeriodSamples * getTimeInterval();
    }

    /**
     * Gets duration of each drift period.
     *
     * @return duration of each drift period.
     */
    public Time getDriftPeriod() {
        return new Time(getDriftPeriodSeconds(), TimeUnit.SECOND);
    }

    /**
     * Gets duration of each drift period.
     *
     * @param result instance where result will be stored.
     */
    public void getDriftPeriod(final Time result) {
        result.setValue(getDriftPeriodSeconds());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates if estimator is ready to start processing additional kinematics
     * measurements.
     *
     * @return true if ready, false otherwise.
     */
    public boolean isReady() {
        return mDriftEstimator.isReady();
    }

    /**
     * Adds a sample of measured body kinematics (accelerometer + gyroscope readings)
     * obtained from an IMU, fixes their values and uses fixed values to estimate
     * any additional existing bias or position and velocity variation while the
     * IMU body remains static.
     *
     * @param kinematics measured body kinematics.
     * @throws LockedException               if estimator is currently running.
     * @throws RandomWalkEstimationException if estimation fails for some reason.
     * @throws NotReadyException             if estimator is not ready.
     */
    public void addBodyKinematics(final BodyKinematics kinematics)
            throws LockedException, RandomWalkEstimationException,
            NotReadyException {

        if (mRunning) {
            throw new LockedException();
        }

        if (!mDriftEstimator.isReady()) {
            throw new NotReadyException();
        }

        try {
            mRunning = true;

            final int numberOfSamples = getNumberOfProcessedSamples();

            if (numberOfSamples == 0) {
                if (mListener != null) {
                    mListener.onStart(this);
                }
            }

            if (mFixKinematics) {
                mFixer.fix(kinematics, mFixedKinematics);
            } else {
                mFixedKinematics.copyFrom(kinematics);
            }

            mBiasEstimator.addBodyKinematics(mFixedKinematics);

            final double timeInterval = getTimeInterval();

            // update random walk values)

            // obtain accumulated mean of bias values for all processed samples.
            // If calibration and sensor was perfect, this should be zero
            final double elapsedTime = getElapsedTimeSeconds();
            mBiasEstimator.getBiasF(mAccelerationTriad);
            mBiasEstimator.getBiasAngularRate(mAngularSpeedTriad);

            final double accelerationBiasNorm = mAccelerationTriad.getNorm();
            final double angularSpeedBiasNorm = mAngularSpeedTriad.getNorm();

            // estimate PSD's of rates of variation of bias values for all processed
            // samples
            mAccelerometerBiasPSD = Math.pow(accelerationBiasNorm / elapsedTime, 2.0)
                    * timeInterval;
            mGyroBiasPSD = Math.pow(angularSpeedBiasNorm / elapsedTime, 2.0)
                    * timeInterval;


            // internally drift estimator will fix kinematics if needed
            mDriftEstimator.addBodyKinematics(kinematics);

            if (numberOfSamples % mDriftPeriodSamples == 0) {
                // for each drift period, update mean and variance values
                // of drift and reset drift estimator
                final double n = mNumberOfProcessedDriftPeriods + 1;
                final double tmp = (double) mNumberOfProcessedDriftPeriods / n;

                final double positionDrift = mDriftEstimator
                        .getCurrentPositionDriftNormMeters();
                final double velocityDrift = mDriftEstimator
                        .getCurrentVelocityDriftNormMetersPerSecond();
                final double attitudeDrift = mDriftEstimator
                        .getCurrentOrientationDriftRadians();

                mAvgPositionDrift = mAvgPositionDrift * tmp + positionDrift / n;
                mAvgVelocityDrift = mAvgVelocityDrift * tmp + velocityDrift / n;
                mAvgAttitudeDrift = mAvgAttitudeDrift * tmp + attitudeDrift / n;

                final double diffPositionDrift = positionDrift - mAvgPositionDrift;
                final double diffVelocityDrift = velocityDrift - mAvgVelocityDrift;
                final double diffAttitudeDrift = attitudeDrift - mAvgAttitudeDrift;

                final double diffPositionDrift2 = diffPositionDrift * diffAttitudeDrift;
                final double diffVelocityDrift2 = diffVelocityDrift * diffVelocityDrift;
                final double diffAttitudeDrift2 = diffAttitudeDrift * diffAttitudeDrift;

                mVarPositionDrift = mVarPositionDrift * tmp + diffPositionDrift2 / n;
                mVarVelocityDrift = mVarVelocityDrift * tmp + diffVelocityDrift2 / n;
                mVarAttitudeDrift = mVarAttitudeDrift * tmp + diffAttitudeDrift2 / n;

                mDriftEstimator.reset();
                mNumberOfProcessedDriftPeriods++;
            }

            if (mListener != null) {
                mListener.onBodyKinematicsAdded(this, kinematics,
                        mFixedKinematics);
            }

        } catch (AlgebraException | DriftEstimationException e) {
            throw new RandomWalkEstimationException(e);
        } finally {
            mRunning = false;
        }
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
        if (mBiasEstimator.reset()) {
            mAccelerometerBiasPSD = 0.0;
            mGyroBiasPSD = 0.0;
            mNumberOfProcessedDriftPeriods = 0;
            mAvgPositionDrift = 0.0;
            mAvgVelocityDrift = 0.0;
            mAvgAttitudeDrift = 0.0;
            mVarPositionDrift = 0.0;
            mVarVelocityDrift = 0.0;
            mVarAttitudeDrift = 0.0;

            if (mListener != null) {
                mListener.onReset(this);
            }

            mRunning = false;
            return true;
        } else {
            mRunning = false;
            return false;
        }
    }

    /**
     * Gets estimated accelerometer bias random walk PSD (Power Spectral Density)
     * expressed in (m^2 * s^-5).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return accelerometer bias random walk PSD.
     */
    @Override
    public double getAccelerometerBiasPSD() {
        return mAccelerometerBiasPSD;
    }

    /**
     * Gets estimated gyro bias random walk PSD (Power Spectral Density)
     * expressed in (rad^2 * s^-3).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return gyroscope gias random walk PSD.
     */
    @Override
    public double getGyroBiasPSD() {
        return mGyroBiasPSD;
    }

    /**
     * Gets estimated position noise variance expressed in squared meters (m^2).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return position variance.
     */
    public double getPositionNoiseVariance() {
        return mVarPositionDrift;
    }

    /**
     * Gets estimated velocity noise variance expressed in (m^2/s^2).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return velocity variance.
     */
    public double getVelocityNoiseVariance() {
        return mVarVelocityDrift;
    }

    /**
     * Gets estimated attitude noise variance expressed in squared radians (rad^2).
     *
     * @return attitude variance.
     */
    public double getAttitudeNoiseVariance() {
        return mVarAttitudeDrift;
    }

    /**
     * Gets standard deviation of position noise expressed in meters (m).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return standard deviation of position noise.
     */
    @Override
    public double getPositionNoiseStandardDeviation() {
        return Math.sqrt(mVarPositionDrift);
    }

    /**
     * Gets standard deviation of position noise.
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return standard deviation of position noise.
     */
    public Distance getPositionNoiseStandardDeviationAsDistance() {
        return new Distance(getPositionNoiseStandardDeviation(),
                DistanceUnit.METER);
    }

    /**
     * Gets standard deviation of position noise.
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionNoiseStandardDeviationAsDistance(
            final Distance result) {
        result.setValue(getPositionNoiseStandardDeviation());
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets standard deviation of velocity noise expressed in meters per second
     * (m/s).
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return standard deviation of velocity noise.
     */
    @Override
    public double getVelocityNoiseStandardDeviation() {
        return Math.sqrt(mVarVelocityDrift);
    }

    /**
     * Gets standard deviation of velocity noise.
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @return standard deviation of velocity noise.
     */
    public Speed getVelocityNoiseStandardDeviationAsSpeed() {
        return new Speed(getVelocityNoiseStandardDeviation(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets standard deviation of velocity noise.
     * This can be used by {@link INSLooselyCoupledKalmanConfig}.
     *
     * @param result instance where result will be stored.
     */
    public void getVelocityNoiseStandardDeviationAsSpeed(final Speed result) {
        result.setValue(getVelocityNoiseStandardDeviation());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets standard deviation of attitude noise expressed in radians (rad).
     *
     * @return standard deviation of attitude noise.
     */
    public double getAttitudeNoiseStandardDeviation() {
        return Math.sqrt(mVarAttitudeDrift);
    }

    /**
     * Gets standard deviation of attitude noise.
     *
     * @return standard deviation of attitude noise.
     */
    public Angle getAttitudeNoiseStandardDeviationAsAngle() {
        return new Angle(getAttitudeNoiseStandardDeviation(), AngleUnit.RADIANS);
    }

    /**
     * Gets standard deviation of attitude noise.
     *
     * @param result instance where result will be stored.
     */
    public void getAttitudeNoiseStandardDeviationAsAngle(final Angle result) {
        result.setValue(getAttitudeNoiseStandardDeviation());
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets position uncertainty expressed in meters (m).
     *
     * @return position uncertainty.
     */
    @Override
    public double getPositionUncertainty() {
        return mAvgPositionDrift;
    }

    /**
     * Gets position uncertainty.
     *
     * @return position uncertainty.
     */
    public Distance getPositionUncertaintyAsDistance() {
        return new Distance(getPositionUncertainty(), DistanceUnit.METER);
    }

    /**
     * Gets position uncertainty.
     *
     * @param result instance where result will be stored.
     */
    public void getPositionUncertaintyAsDistance(final Distance result) {
        result.setValue(getPositionUncertainty());
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets velocity uncertainty expressed in meters per second (m/s).
     *
     * @return velocity uncertainty.
     */
    @Override
    public double getVelocityUncertainty() {
        return mAvgVelocityDrift;
    }

    /**
     * Gets velocity uncertainty.
     *
     * @return velocity uncertainty.
     */
    public Speed getVelocityUncertaintyAsSpeed() {
        return new Speed(getVelocityUncertainty(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets velocity uncertainty.
     *
     * @param result instance where result will be stored.
     */
    public void getVelocityUncertaintyAsSpeed(final Speed result) {
        result.setValue(getVelocityUncertainty());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets attitude uncertainty expressed in radians (rad).
     *
     * @return attitude uncertainty.
     */
    @Override
    public double getAttitudeUncertainty() {
        return mAvgAttitudeDrift;
    }

    /**
     * Gets attitude uncertainty.
     *
     * @return attitude uncertainty.
     */
    public Angle getAttitudeUncertaintyAsAngle() {
        return new Angle(getAttitudeUncertainty(), AngleUnit.RADIANS);
    }

    /**
     * Gets attitude uncertainty.
     *
     * @param result instance where result will be stored.
     */
    public void getAttitudeUncertaintyAsAngle(final Angle result) {
        result.setValue(getAttitudeUncertainty());
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets last added kinematics after being fixed using provided
     * calibration data.
     * If kinematics fix is disabled, this will be equal to the last
     * provided measured kinematics.
     *
     * @return last fixed body kinematics.
     */
    public BodyKinematics getFixedKinematics() {
        return mFixedKinematics;
    }

    /**
     * Gets last added kinematics after being fixed using provided
     * calibration data.
     * If kinematics fix is disabled, this will be equal to the last
     * provided measured kinematics.
     *
     * @param result last fixed body kinematics.
     */
    public void getFixedKinematics(final BodyKinematics result) {
        result.copyFrom(mFixedKinematics);
    }
}
