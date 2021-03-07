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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.units.*;

/**
 * Estimates accumulated drift in body orientation, position and velocity per
 * unit of time.
 * This estimator must be executed while the body where the IMU is placed remains
 * static.
 */
public class DriftEstimator {

    /**
     * Default time interval between kinematics samples expressed in seconds (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Listener to handle events raised by this estimator.
     */
    private DriftEstimatorListener mListener;

    /**
     * Initial frame containing body position, velocity and orientation expressed
     * in ECEF coordinates before starting drift estimation.
     */
    private ECEFFrame mReferenceFrame;

    /**
     * Contains current frame after one navigation step.
     * This is reused for efficiency.
     */
    private final ECEFFrame mFrame = new ECEFFrame();

    /**
     * Fixes body kinematics measurements using accelerometer + gyroscope
     * calibration data to fix measurements.
     */
    private final BodyKinematicsFixer mFixer = new BodyKinematicsFixer();

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
     * Time interval expressed in seconds (s) between body kinematics samples.
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Indicates whether this estimator is running.
     */
    private boolean mRunning;

    /**
     * Contains orientation of reference frame.
     * This is reused for efficiency.
     */
    private final Quaternion mRefQ = new Quaternion();

    /**
     * Contains inverse of orientation of reference frame.
     * This is reused for efficiency.
     */
    private final Quaternion mInvRefQ = new Quaternion();

    /**
     * Contains current frame orientation drift.
     * This is reused for efficiency.
     */
    private final Quaternion mQ = new Quaternion();

    /**
     * Contains current position drift.
     */
    private final ECEFPosition mCurrentPositionDrift = new ECEFPosition();

    /**
     * Contains current velocity drift.
     */
    private final ECEFVelocity mCurrentVelocityDrift = new ECEFVelocity();

    /**
     * Contains current orientation expressed as a 3D rotation matrix.
     */
    private Matrix mCurrentC;

    /**
     * Number of processed body kinematics samples.
     */
    private int mNumberOfProcessedSamples;

    /**
     * Current position drift expressed in meters (m).
     */
    private double mCurrentPositionDriftMeters;

    /**
     * Current velocity drift expressed in meters per second (m/s).
     */
    private double mCurrentVelocityDriftMetersPerSecond;

    /**
     * Current orientation drift expressed in radians (rad).
     */
    private double mCurrentOrientationDriftRadians;

    /**
     * Constructor.
     */
    public DriftEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events.
     */
    public DriftEstimator(final DriftEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     */
    public DriftEstimator(final ECEFFrame referenceFrame) {
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param listener       listener to handle events.
     */
    public DriftEstimator(final ECEFFrame referenceFrame,
                          final DriftEstimatorListener listener) {
        mReferenceFrame = referenceFrame;
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     */
    public DriftEstimator(final NEDFrame referenceFrame) {
        try {
            setReferenceNedFrame(referenceFrame);
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param listener       listener to handle events.
     */
    public DriftEstimator(final NEDFrame referenceFrame,
                          final DriftEstimatorListener listener) {
        this(referenceFrame);
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
    public DriftEstimator(final AccelerationTriad ba,
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
    public DriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
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
    public DriftEstimator(
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
    public DriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
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
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
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
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
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
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
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
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(final ECEFFrame referenceFrame,
                          final AccelerationTriad ba,
                          final Matrix ma,
                          final AngularSpeedTriad bg,
                          final Matrix mg) throws AlgebraException {
        this(ba, ma, bg, mg);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, listener);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg, listener);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        this(ba, ma, bg, mg);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, listener);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg, listener);
        mReferenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(final NEDFrame referenceFrame,
                          final AccelerationTriad ba,
                          final Matrix ma,
                          final AngularSpeedTriad bg,
                          final Matrix mg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, listener);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, gg);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, gg, listener);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, listener);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, gg);
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame),
                ba, ma, bg, mg, gg, listener);
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public DriftEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if estimator is running.
     */
    public void setListener(final DriftEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in ECEF coordinates before starting drift estimation.
     *
     * @return initial body frame or null.
     */
    public ECEFFrame getReferenceFrame() {
        return mReferenceFrame;
    }

    /**
     * Sets initial frame containing body position, velocity and orientation
     * expressed in ECEF coordinates before starting drift estimation.
     *
     * @param referenceFrame initial frame or null.
     * @throws LockedException if estimator is already running.
     */
    public void setReferenceFrame(final ECEFFrame referenceFrame)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mReferenceFrame = referenceFrame;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @return initial body frame or null.
     */
    public NEDFrame getReferenceNedFrame() {
        return mReferenceFrame != null ?
                ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(mReferenceFrame)
                : null;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial frame was available and result was updated, false
     * otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedFrame(final NEDFrame result) {
        if (mReferenceFrame != null) {
            ECEFtoNEDFrameConverter.convertECEFtoNED(mReferenceFrame, result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @param referenceNedFrame initial body frame or null.
     * @throws LockedException if estimator is already running.
     */
    public void setReferenceNedFrame(final NEDFrame referenceNedFrame)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (referenceNedFrame != null) {
            if (mReferenceFrame != null) {
                NEDtoECEFFrameConverter.convertNEDtoECEF(referenceNedFrame, mReferenceFrame);
            } else {
                mReferenceFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                        referenceNedFrame);
            }
        } else {
            mReferenceFrame = null;
        }
    }

    /**
     * Gets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @return initial body position or null.
     */
    public ECEFPosition getReferenceEcefPosition() {
        return mReferenceFrame != null ? mReferenceFrame.getECEFPosition() : null;
    }

    /**
     * Gets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body position was available and result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefPosition(final ECEFPosition result) {
        if (mReferenceFrame != null) {
            mReferenceFrame.getECEFPosition(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param referenceEcefPosition initial body position.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided position is null.
     */
    public void setReferenceEcefPosition(final ECEFPosition referenceEcefPosition)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mReferenceFrame != null) {
            mReferenceFrame.setPosition(referenceEcefPosition);
        } else {
            mReferenceFrame = new ECEFFrame(referenceEcefPosition);
        }
    }

    /**
     * Gets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @return initial body velocity or null.
     */
    public ECEFVelocity getReferenceEcefVelocity() {
        return mReferenceFrame != null ? mReferenceFrame.getECEFVelocity() : null;
    }

    /**
     * Gets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body velocity was available and result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefVelocity(final ECEFVelocity result) {
        if (mReferenceFrame != null) {
            mReferenceFrame.getECEFVelocity(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param referenceEcefVelocity initial body velocity.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided velocity is null.
     */
    public void setReferenceEcefVelocity(final ECEFVelocity referenceEcefVelocity)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mReferenceFrame == null) {
            mReferenceFrame = new ECEFFrame();
        }

        mReferenceFrame.setVelocity(referenceEcefVelocity);
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @return initial body orientation or null.
     */
    public CoordinateTransformation getReferenceEcefCoordinateTransformation() {
        return mReferenceFrame != null ?
                mReferenceFrame.getCoordinateTransformation() : null;
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body orientation was available and result was
     * updated, false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefCoordinateTransformation(
            final CoordinateTransformation result) {
        if (mReferenceFrame != null) {
            mReferenceFrame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @param referenceEcefCoordinateTransformation initial body orientation.
     * @throws LockedException                               if estimator is already running.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided source and
     *                                                       destination types are invalid. Source type must be
     *                                                       {@link com.irurueta.navigation.frames.FrameType#BODY_FRAME} and destination
     *                                                       type must be {@link com.irurueta.navigation.frames.FrameType#EARTH_CENTERED_EARTH_FIXED_FRAME}
     *                                                       indicating that body orientation is expressed respect ECEF coordinates.
     * @throws NullPointerException                          if provided orientation is null.
     */
    public void setReferenceEcefCoordinateTransformation(
            final CoordinateTransformation referenceEcefCoordinateTransformation)
            throws LockedException, InvalidSourceAndDestinationFrameTypeException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mReferenceFrame == null) {
            mReferenceFrame = new ECEFFrame(referenceEcefCoordinateTransformation);
        } else {
            mReferenceFrame.setCoordinateTransformation(
                    referenceEcefCoordinateTransformation);
        }
    }

    /**
     * Gets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @return initial body position or null.
     */
    public NEDPosition getReferenceNedPosition() {
        final NEDFrame nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getPosition() : null;
    }

    /**
     * Gets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body position was available and result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedPosition(final NEDPosition result) {
        if (mReferenceFrame != null) {
            final NEDFrame nedFrame = getReferenceNedFrame();
            nedFrame.getPosition(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param referenceNedPosition initial body position.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided position is null.
     */
    public void setReferenceNedPosition(final NEDPosition referenceNedPosition)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mReferenceFrame != null) {
            final NEDFrame nedFrame = getReferenceNedFrame();
            nedFrame.setPosition(referenceNedPosition);
            setReferenceNedFrame(nedFrame);
        } else {
            setReferenceNedFrame(new NEDFrame(referenceNedPosition));
        }
    }

    /**
     * Gets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @return initial body velocity or null.
     */
    public NEDVelocity getReferenceNedVelocity() {
        final NEDFrame nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getVelocity() : null;
    }

    /**
     * Gets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body velocity was available and result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedVelocity(final NEDVelocity result) {
        if (mReferenceFrame != null) {
            final NEDFrame nedFrame = getReferenceNedFrame();
            nedFrame.getVelocity(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param referenceNedVelocity initial body velocity.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided velocity is null.
     */
    public void setReferenceNedVelocity(final NEDVelocity referenceNedVelocity)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame;
        if (mReferenceFrame != null) {
            nedFrame = getReferenceNedFrame();
        } else {
            nedFrame = new NEDFrame();
        }

        nedFrame.setVelocity(referenceNedVelocity);
        setReferenceNedFrame(nedFrame);
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @return initial body orientation or null.
     */
    public CoordinateTransformation getReferenceNedCoordinateTransformation() {
        final NEDFrame nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getCoordinateTransformation() : null;
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @param result instance where result will be stored.
     * @return true if initial body orientation was available and result was
     * updated, false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedCoordinateTransformation(
            final CoordinateTransformation result) {
        if (mReferenceFrame != null) {
            final NEDFrame nedFrame = getReferenceNedFrame();
            nedFrame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @param referenceNedCoordinateTransformation initial body orientation.
     * @throws LockedException                               if estimator is already running.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided source and
     *                                                       destination types are invalid. Source type must be
     *                                                       {@link com.irurueta.navigation.frames.FrameType#BODY_FRAME} and destination
     *                                                       type must be {@link com.irurueta.navigation.frames.FrameType#LOCAL_NAVIGATION_FRAME}
     *                                                       indicating that body orientation is expressed respect NED coordinates.
     * @throws NullPointerException                          if provided orientation is null.
     */
    public void setReferenceNedCoordinateTransformation(
            final CoordinateTransformation referenceNedCoordinateTransformation)
            throws LockedException, InvalidSourceAndDestinationFrameTypeException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame;
        if (mReferenceFrame == null) {
            nedFrame = new NEDFrame(referenceNedCoordinateTransformation);
            setReferenceNedFrame(nedFrame);
        } else {
            nedFrame = getReferenceNedFrame();
            nedFrame.setCoordinateTransformation(
                    referenceNedCoordinateTransformation);
        }
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
    public void setAccelerationBiasX(final Acceleration biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFixer.setAccelerationBiasX(biasX);
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
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Indicates if estimator is ready to start processing additional kinematics
     * measurements.
     *
     * @return true if ready, false otherwise.
     */
    public boolean isReady() {
        return mReferenceFrame != null;
    }

    /**
     * Indicates whether this estimator is running.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Adds a sample of measured body kinematics (accelerometer + gyroscope readings)
     * obtained from an IMU, fixes their values and uses fixed values to estimate
     * current drift and their average values.
     *
     * @param kinematics measured body kinematics.
     * @throws LockedException          if estimator is currently running.
     * @throws NotReadyException        if estimator is not ready.
     * @throws DriftEstimationException if estimation fails for some reason.
     */
    public void addBodyKinematics(final BodyKinematics kinematics)
            throws LockedException, NotReadyException, DriftEstimationException {
        if (mRunning) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mRunning = true;

            if (mNumberOfProcessedSamples == 0) {
                if (mListener != null) {
                    mListener.onStart(this);
                }

                final CoordinateTransformation c = mReferenceFrame
                        .getCoordinateTransformation();
                c.asRotation(mRefQ);
                mRefQ.inverse(mInvRefQ);

                mFrame.copyFrom(mReferenceFrame);
            }

            if (mFixKinematics) {
                mFixer.fix(kinematics, mFixedKinematics);
            } else {
                mFixedKinematics.copyFrom(kinematics);
            }

            // estimate navigation variation respect to previous frame
            ECEFInertialNavigator.navigateECEF(mTimeInterval, mFrame,
                    mFixedKinematics, mFrame);

            // estimate drift averages
            computeCurrentPositionDrift();
            computeCurrentVelocityDrift();
            computeCurrentOrientationDrift();

            mNumberOfProcessedSamples++;

            if (mListener != null) {
                mListener.onBodyKinematicsAdded(this, kinematics,
                        mFixedKinematics);
            }

        } catch (final AlgebraException | InertialNavigatorException |
                InvalidRotationMatrixException e) {
            throw new DriftEstimationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Resets this estimator to its initial state.
     *
     * @throws LockedException if estimator is currently running.
     */
    public void reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;
        mNumberOfProcessedSamples = 0;

        mCurrentPositionDriftMeters = 0.0;
        mCurrentVelocityDriftMetersPerSecond = 0.0;
        mCurrentOrientationDriftRadians = 0.0;

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;
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
     * Gets elapsed time since first processed measurement expressed in seconds.
     *
     * @return elapsed time.
     */
    public double getElapsedTimeSeconds() {
        return mNumberOfProcessedSamples * mTimeInterval;
    }

    /**
     * Gets elapsed time since first processed measurement.
     *
     * @return elapsed time.
     */
    public Time getElapsedTime() {
        return mNumberOfProcessedSamples > 0 ?
                new Time(getElapsedTimeSeconds(), TimeUnit.SECOND) : null;
    }

    /**
     * Gets elapsed time since first processed measurement.
     *
     * @param result instance where result will be stored.
     * @return true if elapsed time is available and result is updated,
     * false otherwise.
     */
    public boolean getElapsedTime(final Time result) {
        if (mNumberOfProcessedSamples > 0) {
            result.setValue(getElapsedTimeSeconds());
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current position drift for last processed body kinematics
     * measurement expressed in meters (m) respect ECEF coordinates.
     *
     * @return current position drift or null.
     */
    public ECEFPosition getCurrentPositionDrift() {
        return mNumberOfProcessedSamples > 0 ?
                new ECEFPosition(mCurrentPositionDrift) : null;
    }

    /**
     * Gets current position drift for last processed body kinematics
     * measurement expressed in meters (m) respect ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if current position drift is available and result is updated,
     * false otherwise.
     */
    public boolean getCurrentPositionDrift(final ECEFPosition result) {
        if (mNumberOfProcessedSamples > 0) {
            result.copyFrom(mCurrentPositionDrift);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current velocity drift for last processed body kinematics
     * measurement expressed in meters per second (m/s) respect ECEF coordinates.
     *
     * @return current velocity drift or null.
     */
    public ECEFVelocity getCurrentVelocityDrift() {
        return mNumberOfProcessedSamples > 0 ?
                new ECEFVelocity(mCurrentVelocityDrift) : null;
    }

    /**
     * Gets current velocity drift for last processed body kinematics
     * measurement expressed in meters per second (m/s) respect ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if current velocity drift is available and result is updated,
     * false otherwise.
     */
    public boolean getCurrentVelocityDrift(final ECEFVelocity result) {
        if (mNumberOfProcessedSamples > 0) {
            result.copyFrom(mCurrentVelocityDrift);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current orientation drift as a 3D rotation for last processed body
     * kinematics measurement respect ECEF coordinates.
     *
     * @return current orientation drift or null.
     */
    public Rotation3D getCurrentOrientationDrift() {
        return mNumberOfProcessedSamples > 0 ?
                new Quaternion(mQ) : null;
    }

    /**
     * Gets current orientation drift as a 3D rotation for last processed body
     * kinematics measurement respect ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if current orientation drift is available and result is
     * updated, false otherwise.
     */
    public boolean getCurrentOrientationDrift(final Rotation3D result) {
        if (mNumberOfProcessedSamples > 0) {
            result.fromRotation(mQ);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement expressed in meters (m).
     *
     * @return current amount of position drift or null.
     */
    public Double getCurrentPositionDriftNormMeters() {
        return mNumberOfProcessedSamples > 0 ?
                mCurrentPositionDriftMeters : null;
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement.
     *
     * @return current amount of position drift or null.
     */
    public Distance getCurrentPositionDriftNorm() {
        final Double positionDrift = getCurrentPositionDriftNormMeters();
        return positionDrift != null ?
                new Distance(positionDrift, DistanceUnit.METER) : null;
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where result will be stored.
     * @return true if current position drift is available and result is updated,
     * false otherwise.
     */
    public boolean getCurrentPositionDriftNorm(final Distance result) {
        final Double positionDrift = getCurrentPositionDriftNormMeters();
        if (positionDrift != null) {
            result.setValue(positionDrift);
            result.setUnit(DistanceUnit.METER);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of velocity drift for last processed body kinematics
     * measurement expressed in meters per second (m/s).
     *
     * @return current amount of velocity drift or null.
     */
    public Double getCurrentVelocityDriftNormMetersPerSecond() {
        return mNumberOfProcessedSamples > 0 ?
                mCurrentVelocityDriftMetersPerSecond : null;
    }

    /**
     * Gets current amount of velocity drift for last processed body kinematics
     * measurement.
     *
     * @return current amount of velocity drift or null.
     */
    public Speed getCurrentVelocityDriftNorm() {
        final Double velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        return velocityDrift != null ?
                new Speed(velocityDrift, SpeedUnit.METERS_PER_SECOND) : null;
    }

    /**
     * Gets current amount of velocity drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where result will be stored.
     * @return true if current velocity drift is available and result is updated,
     * false otherwise.
     */
    public boolean getCurrentVelocityDriftNorm(final Speed result) {
        final Double velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        if (velocityDrift != null) {
            result.setValue(velocityDrift);
            result.setUnit(SpeedUnit.METERS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement expressed in radians (rad).
     *
     * @return current amount of orientation drift or null.
     */
    public Double getCurrentOrientationDriftRadians() {
        return mNumberOfProcessedSamples > 0 ?
                mCurrentOrientationDriftRadians : null;
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement.
     *
     * @return current amount of orientation drift or null.
     */
    public Angle getCurrentOrientationDriftAngle() {
        final Double orientationDrift = getCurrentOrientationDriftRadians();
        return orientationDrift != null ?
                new Angle(orientationDrift, AngleUnit.RADIANS) : null;
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where result will be stored.
     * @return true if current orientation drift is available and result is
     * updated, false otherwise.
     */
    public boolean getCurrentOrientationDriftAngle(final Angle result) {
        final Double orientationDrift = getCurrentOrientationDriftRadians();
        if (orientationDrift != null) {
            result.setValue(orientationDrift);
            result.setUnit(AngleUnit.RADIANS);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of position drift per time unit expressed in meters
     * per second (m/s).
     *
     * @return current amount of position drift per time unit or null.
     */
    public Double getCurrentPositionDriftPerTimeUnit() {
        final Double positionDrift = getCurrentPositionDriftNormMeters();
        final double elapsedTime = getElapsedTimeSeconds();
        return positionDrift != null ? positionDrift / elapsedTime : null;
    }

    /**
     * Gets current amount of position drift per time unit.
     *
     * @return current amount of position drift per time unit or null.
     */
    public Speed getCurrentPositionDriftPerTimeUnitAsSpeed() {
        final Double positionDriftPerTimeUnit = getCurrentPositionDriftPerTimeUnit();
        return positionDriftPerTimeUnit != null
                ? new Speed(positionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)
                : null;
    }

    /**
     * Gets current amount of position drift per time unit.
     *
     * @param result instance where result will be stored.
     * @return true if current amount of position drift per time unit is
     * available and result is updated, false otherwise.
     */
    public boolean getCurrentPositionDriftPerTimeUnitAsSpeed(final Speed result) {
        final Double positionDriftPerTimeUnit = getCurrentPositionDriftPerTimeUnit();
        if (positionDriftPerTimeUnit != null) {
            result.setValue(positionDriftPerTimeUnit);
            result.setUnit(SpeedUnit.METERS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of velocity drift per time unit expressed in meters
     * per squared second (m/s^2).
     *
     * @return current amount of velocity drift per time unit or null.
     */
    public Double getCurrentVelocityDriftPerTimeUnit() {
        final Double velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        final double elapsedTime = getElapsedTimeSeconds();
        return velocityDrift != null ? velocityDrift / elapsedTime : null;
    }

    /**
     * Gets current amount of velocity drift per time unit.
     *
     * @return current amount of velocity drift per time unit or null.
     */
    public Acceleration getCurrentVelocityDriftPerTimeUnitAsAcceleration() {
        final Double velocityDriftPerTimeUnit = getCurrentVelocityDriftPerTimeUnit();
        return velocityDriftPerTimeUnit != null
                ? new Acceleration(velocityDriftPerTimeUnit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets current amount of velocity drift per time unit.
     *
     * @param result instance where result will be stored.
     * @return true if current amount of velocity drift per time unit is available
     * and result is updated, false otherwise.
     */
    public boolean getCurrentVelocityDriftPerTimeUnitAsAcceleration(
            final Acceleration result) {
        final Double velocityDriftPerTimeUnit = getCurrentVelocityDriftPerTimeUnit();
        if (velocityDriftPerTimeUnit != null) {
            result.setValue(velocityDriftPerTimeUnit);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of orientation drift per time unit expressed in radians
     * per second (rad/s).
     *
     * @return amount of orientation drift per time unit or null.
     */
    public Double getCurrentOrientationDriftPerTimeUnit() {
        final Double orientationDrift = getCurrentOrientationDriftRadians();
        final double elapsedTime = getElapsedTimeSeconds();
        return orientationDrift != null ? orientationDrift / elapsedTime : null;
    }

    /**
     * Gets current amount of orientation drift per time unit.
     *
     * @return amount of orientation drift per time unit or null.
     */
    public AngularSpeed getCurrentOrientationDriftPerTimeUnitAsAngularSpeed() {
        final Double orientationDriftPerTimeUnit = getCurrentOrientationDriftPerTimeUnit();
        return orientationDriftPerTimeUnit != null ?
                new AngularSpeed(orientationDriftPerTimeUnit,
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets current amount of orientation drift per time unit.
     *
     * @param result instance where result will be stored.
     * @return true if current amount of orientation drift is available and result
     * is updated, false otherwise.
     */
    public boolean getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
            final AngularSpeed result) {
        final Double orientationDriftPerTimeUnit = getCurrentOrientationDriftPerTimeUnit();
        if (orientationDriftPerTimeUnit != null) {
            result.setValue(orientationDriftPerTimeUnit);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Computes current position drift.
     */
    private void computeCurrentPositionDrift() {
        final double initX = mReferenceFrame.getX();
        final double initY = mReferenceFrame.getY();
        final double initZ = mReferenceFrame.getZ();

        final double currentX = mFrame.getX();
        final double currentY = mFrame.getY();
        final double currentZ = mFrame.getZ();

        final double diffX = currentX - initX;
        final double diffY = currentY - initY;
        final double diffZ = currentZ - initZ;

        mCurrentPositionDrift.setCoordinates(diffX, diffY, diffZ);

        mCurrentPositionDriftMeters = mCurrentPositionDrift.getNorm();
    }

    /**
     * Computes current velocity drift.
     */
    private void computeCurrentVelocityDrift() {
        final double initVx = mReferenceFrame.getVx();
        final double initVy = mReferenceFrame.getVy();
        final double initVz = mReferenceFrame.getVz();

        final double currentVx = mFrame.getVx();
        final double currentVy = mFrame.getVy();
        final double currentVz = mFrame.getVz();

        final double diffVx = currentVx - initVx;
        final double diffVy = currentVy - initVy;
        final double diffVz = currentVz - initVz;

        mCurrentVelocityDrift.setCoordinates(diffVx, diffVy, diffVz);

        mCurrentVelocityDriftMetersPerSecond = mCurrentVelocityDrift.getNorm();
    }

    /**
     * Computes current orientation drift.
     *
     * @throws AlgebraException               if there are numerical instabilities.
     * @throws InvalidRotationMatrixException if rotation cannot be accurately
     *                                        estimated.
     */
    private void computeCurrentOrientationDrift() throws AlgebraException,
            InvalidRotationMatrixException {
        if (mCurrentC == null) {
            mCurrentC = new Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS);
        }
        mFrame.getCoordinateTransformationMatrix(mCurrentC);

        mQ.fromMatrix(mCurrentC);
        mQ.combine(mInvRefQ);

        mCurrentOrientationDriftRadians = mQ.getRotationAngle();
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
}
