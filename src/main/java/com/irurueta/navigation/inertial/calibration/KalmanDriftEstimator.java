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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanFilteredEstimator;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanState;

/**
 * Estimates accumulated drift in body orientation, position and velocity per
 * unit of time using an INS Kalman filtered solution.
 */
public class KalmanDriftEstimator extends DriftEstimator {

    /**
     * Configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     */
    private INSLooselyCoupledKalmanConfig mKalmanConfig;

    /**
     * Configuration parameters determining initial system noise covariance matrix.
     */
    private INSLooselyCoupledKalmanInitializerConfig mInitConfig;

    /**
     * Internal INS loosely coupled Kalman filtered estimator to estimate
     * accumulated body position, velocity and orientation.
     */
    private INSLooselyCoupledKalmanFilteredEstimator mKalmanEstimator;

    /**
     * Current Kalman filter state.
     */
    private INSLooselyCoupledKalmanState mState;

    /**
     * Constructor.
     */
    public KalmanDriftEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events.
     */
    public KalmanDriftEstimator(final DriftEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     */
    public KalmanDriftEstimator(
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        super(referenceFrame);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(referenceFrame, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        super(referenceFrame);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(referenceFrame, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
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
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        mKalmanConfig = kalmanConfig;
        mInitConfig = initConfig;
    }

    /**
     * Gets configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     *
     * @return configuration parameters for Kalman filter or null.
     */
    public INSLooselyCoupledKalmanConfig getKalmanConfig() {
        return mKalmanConfig;
    }

    /**
     * Sets configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     *
     * @param kalmanConfig configuration parameters for Kalman filter.
     * @throws LockedException if estimator is already running.
     */
    public void setKalmanConfig(final INSLooselyCoupledKalmanConfig kalmanConfig)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mKalmanConfig = kalmanConfig;
    }

    /**
     * Gets configuration parameters determining initial system noise covariance
     * matrix.
     *
     * @return configuration parameters determining initial system noise covariance
     * matrix or null.
     */
    public INSLooselyCoupledKalmanInitializerConfig getInitConfig() {
        return mInitConfig;
    }

    /**
     * Sets configuration parameters determining initial system noise covariance
     * matrix.
     *
     * @param initConfig configuration parameters determining initial system noise
     *                   covariance matrix.
     * @throws LockedException if estimator is already running.
     */
    public void setInitConfig(final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitConfig = initConfig;
    }

    /**
     * Indicates if estimator is ready to start processing additional kinematics
     * measurements.
     *
     * @return true if ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mKalmanConfig != null && mInitConfig != null;
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
    @Override
    public void addBodyKinematics(final BodyKinematics kinematics)
            throws LockedException, NotReadyException, DriftEstimationException {
        if (isRunning()) {
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

                initialize();
            }

            if (mFixKinematics) {
                // fix kinematics and update kalman filter
                mFixer.fix(kinematics, mFixedKinematics);
            } else {
                // only update kalman filter
                mFixedKinematics.copyFrom(kinematics);
            }

            final double timestamp = getElapsedTimeSeconds();
            mKalmanEstimator.update(mFixedKinematics, timestamp);

            if (mState == null) {
                mState = mKalmanEstimator.getState();
            } else {
                mKalmanEstimator.getState(mState);
            }

            // estimate drift values
            computeCurrentPositionDrift();
            computeCurrentVelocityDrift();
            computeCurrentOrientationDrift();

            mNumberOfProcessedSamples++;

            if (mListener != null) {
                mListener.onBodyKinematicsAdded(this,
                        kinematics, mFixedKinematics);
            }

        } catch (final AlgebraException | InvalidRotationMatrixException
                | INSException e) {
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
    @Override
    public void reset() throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        mRunning = true;
        mKalmanEstimator = null;
        mState = null;
        mRunning = false;

        super.reset();
    }

    /**
     * Gets state of internal Kalman filter containing current body position,
     * orientation and velocity after last provided body kinematics measurement.
     *
     * @return state of internal Kalman filter.
     */
    public INSLooselyCoupledKalmanState getState() {
        return mKalmanEstimator != null ? mKalmanEstimator.getState() : null;
    }

    /**
     * Gets state of internal Kalman filter containing current body position,
     * orientation and velocity after last provided body kinematics measurement.
     *
     * @param result instance where result will be stored.
     * @return true if internal Kalman filter state is available and result is
     * updated, false otherwise.
     */
    public boolean getState(final INSLooselyCoupledKalmanState result) {
        if (mKalmanEstimator != null) {
            return mKalmanEstimator.getState(result);
        } else {
            return false;
        }
    }

    /**
     * Initializes internal frame and Kalman estimator when the first body kinematics
     * measurement is provided.
     *
     * @throws InvalidRotationMatrixException if orientation cannot be determined for
     *                                        numerical reasons.
     */
    private void initialize() throws InvalidRotationMatrixException {
        final CoordinateTransformation c = mReferenceFrame
                .getCoordinateTransformation();
        c.asRotation(mRefQ);
        mRefQ.inverse(mInvRefQ);

        mFrame.copyFrom(mReferenceFrame);

        mKalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(
                mKalmanConfig, mInitConfig, mFrame);
    }

    /**
     * Computes current position drift.
     */
    private void computeCurrentPositionDrift() {
        final double initX = mReferenceFrame.getX();
        final double initY = mReferenceFrame.getY();
        final double initZ = mReferenceFrame.getZ();

        final double currentX = mState.getX();
        final double currentY = mState.getY();
        final double currentZ = mState.getZ();

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

        final double currentVx = mState.getVx();
        final double currentVy = mState.getVy();
        final double currentVz = mState.getVz();

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
        mCurrentC = mState.getBodyToEcefCoordinateTransformationMatrix();

        mQ.fromMatrix(mCurrentC);
        mQ.combine(mInvRefQ);

        mCurrentOrientationDriftRadians = mQ.getRotationAngle();
    }
}
