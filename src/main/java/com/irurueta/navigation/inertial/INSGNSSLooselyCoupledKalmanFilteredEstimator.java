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
package com.irurueta.navigation.inertial;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSException;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Calculates position, velocity, attitude and IMU biases using a GNSS unweighted
 * iterated least squares estimator along with an INS loosely coupled Kalman filter
 * to take into account intertial measurements to smooth results.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Loosely_coupled_INS_GNSS.m
 */
public class INSGNSSLooselyCoupledKalmanFilteredEstimator {

    /**
     * Internal estimator to compute least squares solution for GNSS measurements.
     */
    private final GNSSLeastSquaresPositionAndVelocityEstimator mLsEstimator
            = new GNSSLeastSquaresPositionAndVelocityEstimator();

    /**
     * Internal INS estimator to update kinematic measures and propagate
     * estimated state.
     */
    private final INSLooselyCoupledKalmanFilteredEstimator mInsEstimator =
            new INSLooselyCoupledKalmanFilteredEstimator(
                    new INSLooselyCoupledKalmanFilteredEstimatorListener() {
                        @Override
                        public void onUpdateStart(
                                final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                            if (mListener != null) {
                                mListener.onUpdateBodyKinematicsStart(
                                        INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                            }
                        }

                        @Override
                        public void onUpdateEnd(
                                final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                            if (mListener != null) {
                                mListener.onUpdateBodyKinematicsEnd(
                                        INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                            }
                        }

                        @Override
                        public void onPropagateStart(
                                final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                            if (mListener != null) {
                                mListener.onPropagateStart(
                                        INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                            }
                        }

                        @Override
                        public void onPropagateEnd(
                                final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                            if (mListener != null) {
                                mListener.onPropagateEnd(
                                        INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                            }
                        }

                        @Override
                        public void onReset(
                                final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                        }
                    });

    /**
     * Listener to notify events raised by this instance.
     */
    private INSGNSSLooselyCoupledKalmanFilteredEstimatorListener mListener;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> mMeasurements;

    /**
     * Last provided user kinematics containing applied specific force
     * and angular rates resolved in body axes.
     */
    private BodyKinematics mKinematics;

    /**
     * Contains last provided user kinematics minus currently estimated bias
     * for acceleration and angular rate values.
     */
    private BodyKinematics mCorrectedKinematics;

    /**
     * Internally keeps user position, velocity and attitude.
     */
    private ECEFFrame mFrame;

    /**
     * Current Kalman filter state containing current INS estimation along with
     * Kalman filter covariance error matrix.
     */
    private INSLooselyCoupledKalmanState mState;

    /**
     * Current estimation containing user ECEF position, user ECEF velocity, clock offset
     * and clock drift.
     */
    private GNSSEstimation mEstimation;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator() {
    }

    /**
     * Constructor.
     *
     * @param config INS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config) {
        try {
            mInsEstimator.setConfig(config);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval) {
        try {
            mInsEstimator.setEpochInterval(epochInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval) {
        this(epochInterval);
        try {
            mInsEstimator.setConfig(config);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final Time epochInterval) {
        this(TimeConverter.convert(epochInterval.getValue().doubleValue(),
                epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval) {
        this(config, TimeConverter.convert(epochInterval.getValue().doubleValue(),
                epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param c body-to-ECEF coordinate transformation defining the initial body
     *          attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this();
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config INS loosely Kalman filter configuration parameters (usually obtained
     *               through calibration).
     * @param c      body-to-ECEF coordinate transformation defining the initial body
     *               attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        try {
            mInsEstimator.setConfig(config);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param c        body-to-ECEF coordinate transformation defining the initial body
     *                 attitude.
     * @param listener listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param c        body-to-ECEF coordinate transformation defining the initial body
     *                 attitude.
     * @param listener listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the
     *                      initial body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this();
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this(config);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this(epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(initialConfig);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this(config, epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this(epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
        this(config, epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum epoch interval expressed in seconds (s) between consecutive
     * propagations or measurements expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval between consecutive propagations or
     * measurements.
     */
    public double getEpochInterval() {
        return mInsEstimator.getEpochInterval();
    }

    /**
     * Sets minimum epoch interval expressed in seconds (s) between consecutive
     * propagations or measurements expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public void setEpochInterval(final double epochInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInsEstimator.setEpochInterval(epochInterval);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param result instance where minimum epoch interval will be stored.
     */
    public void getEpochIntervalAsTime(final Time result) {
        mInsEstimator.getEpochIntervalAsTime(result);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval.
     */
    public Time getEpochIntervalAsTime() {
        return mInsEstimator.getEpochIntervalAsTime();
    }

    /**
     * Sets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param epochInterval minimum epoch interval.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public void setEpochInterval(final Time epochInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInsEstimator.setEpochInterval(epochInterval);
    }

    /**
     * Gets INS loosely coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param result instance where INS loosely coupled Kalman configuration
     *               parameters will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getConfig(final INSLooselyCoupledKalmanConfig result) {
        return mInsEstimator.getConfig(result);
    }

    /**
     * Gets INS loosely coupled Kalman configuration parameters (usually obtained
     * through calibration).
     *
     * @return INS loosely coupled Kalman configuration parameters.
     */
    public INSLooselyCoupledKalmanConfig getConfig() {
        return mInsEstimator.getConfig();
    }

    /**
     * Sets INS loosely coupled Kalman configuration parameters (usually obtained
     * through calibration).
     *
     * @param config INS loosely coupled Kalman configuration parameters to be
     *               set.
     * @throws LockedException if this estimator is already runnign.
     */
    public void setConfig(final INSLooselyCoupledKalmanConfig config)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInsEstimator.setConfig(config);
    }

    /**
     * Gets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @return body-to-ECEF coordinate transformation.
     */
    public CoordinateTransformation getCoordinateTransformation() {
        return mFrame != null ? mFrame.getCoordinateTransformation() : null;
    }

    /**
     * Gets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @param result instance where body-to-ECEF data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getCoordinateTransformation(final CoordinateTransformation result) {
        if (mFrame != null) {
            mFrame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @param c body-to-ECEF coordinate transformation to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid (is not a
     *                                                       body-to-ECEF transformation).
     * @throws LockedException                               if this estimator is already running.
     */
    public void setCoordinateTransformation(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        initFrame();
        mFrame.setCoordinateTransformation(c);
        mInsEstimator.setFrame(mFrame);
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param result instance where configuration data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getInitialConfig(
            final INSLooselyCoupledKalmanInitializerConfig result) {
        return mInsEstimator.getInitialConfig(result);
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration te set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @return initial INS loosely coupled Kalman configuration.
     */
    public INSLooselyCoupledKalmanInitializerConfig getInitialConfig() {
        return mInsEstimator.getInitialConfig();
    }

    /**
     * Sets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param initialConfig initial configuration to be set.
     * @throws LockedException if this estimator is already runnign.
     */
    public void setInitialConfig(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInsEstimator.setInitialConfig(initialConfig);
    }

    /**
     * Gets last updated GNSS measurements of a collection of satellites.
     *
     * @return last updated GNSS measurements of a collection of satellites.
     */
    public Collection<GNSSMeasurement> getMeasurements() {
        if (mMeasurements == null) {
            return null;
        }

        final List<GNSSMeasurement> result = new ArrayList<>();
        for (GNSSMeasurement measurement : mMeasurements) {
            result.add(new GNSSMeasurement(measurement));
        }
        return result;
    }

    /**
     * Gets last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     *
     * @return last provided user kinematics.
     */
    public BodyKinematics getKinematics() {
        final BodyKinematics result = mInsEstimator.getKinematics();
        if (result != null) {
            return result;
        } else {
            if (mKinematics != null) {
                return new BodyKinematics(mKinematics);
            } else {
                return null;
            }
        }
    }

    /**
     * Gets last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getKinematics(final BodyKinematics result) {
        if (!mInsEstimator.getKinematics(result)) {
            if (mKinematics != null) {
                result.copyFrom(mKinematics);
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Gets corrected kinematics which are the last provided user kinematics after
     * removal of the biases estimated by the Kalman filter.
     *
     * @return corrected kinematics.
     * @see #getKinematics()
     */
    public BodyKinematics getCorrectedKinematics() {
        final BodyKinematics result = mInsEstimator.getCorrectedKinematics();
        if (result != null) {
            return result;
        } else {
            if (mCorrectedKinematics != null) {
                return new BodyKinematics(mCorrectedKinematics);
            } else {
                return null;
            }
        }
    }

    /**
     * Gets corrected kinematics which are the last provided user kinematics after
     * removal of the biases estimated by the Kalman filter.
     *
     * @param result instance where corrected body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getCorrectedKinematics(final BodyKinematics result) {
        if (!mInsEstimator.getCorrectedKinematics(result)) {
            if (mCorrectedKinematics != null) {
                result.copyFrom(mCorrectedKinematics);
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     *
     * @return current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     */
    public GNSSEstimation getEstimation() {
        return mEstimation != null ? new GNSSEstimation(mEstimation) : null;
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     * This method does not update result instance if no estimation is available.
     *
     * @param result instance where estimation will be stored.
     * @return true if result estimation was updated, false otherwise.
     */
    public boolean getEstimation(final GNSSEstimation result) {
        if (mEstimation != null) {
            result.copyFrom(mEstimation);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current Kalman filter state containing current user position,
     * velocity, attitude and IMU biases along with Kalman filter
     * covariance error matrix.
     *
     * @return current Kalman filter state.
     */
    public INSLooselyCoupledKalmanState getState() {
        return mState != null ? new INSLooselyCoupledKalmanState(mState) : null;
    }

    /**
     * Gets current Kalman filter state containing current user position,
     * velocity, attitude and IMU biases along with Kalman filter
     * covariance error matrix.
     *
     * @param result instance where state will be stored.
     * @return true if result state was updated, false otherwise.
     */
    public boolean getState(final INSLooselyCoupledKalmanState result) {
        if (mState != null) {
            result.copyFrom(mState);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated.
     *
     * @return timestamp expressed in seconds since epoch time when Kalman filter
     * state was last propagated.
     */
    public Double getLastStateTimestamp() {
        return mInsEstimator.getLastStateTimestamp();
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was las propagated.
     *
     * @param result instance where timestamp since epoch tiem when Kalman filter
     *               state was last propagated will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastStateTimestampAsTime(final Time result) {
        return mInsEstimator.getLastStateTimestampAsTime(result);
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @return timestamp since epoch time when Kalman fitler state was last
     * propagated.
     */
    public Time getLastStateTimestampAsTime() {
        return mInsEstimator.getLastStateTimestampAsTime();
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning || mInsEstimator.isRunning();
    }

    /**
     * Indicates whether provided measurements are ready to be
     * used for an update.
     *
     * @param measurements measurements to be checked.
     * @return true if estimator is ready, false otherwise.
     */
    public static boolean isUpdateMeasurementsReady(
            final Collection<GNSSMeasurement> measurements) {
        return GNSSLeastSquaresPositionAndVelocityEstimator
                .isValidMeasurements(measurements);
    }

    /**
     * Updates GNSS measurements of this estimator when new satellite measurements
     * are available.
     * Calls to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp since epoch time when GNSS measurements were
     *                     updated.
     * @return true if measurements were updated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSGNSSException  if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(
            final Collection<GNSSMeasurement> measurements, final Time timestamp)
            throws LockedException, NotReadyException, INSGNSSException {
        return updateMeasurements(measurements, TimeConverter.convert(
                timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Updates GNSS measurements of this estimator when new satellite measurements
     * are available.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp expressed in seconds since epoch time when
     *                     GNSS measurements were updated.
     * @return true if measurements were updated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSGNSSException  if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(
            final Collection<GNSSMeasurement> measurements, final double timestamp)
            throws LockedException, NotReadyException, INSGNSSException {

        if (mRunning) {
            throw new LockedException();
        }

        if (!isUpdateMeasurementsReady(measurements)) {
            throw new NotReadyException();
        }

        final Double lastStateTimestamp = mInsEstimator.getLastStateTimestamp();
        if (lastStateTimestamp != null &&
                timestamp - lastStateTimestamp <= mInsEstimator.getEpochInterval()) {
            return false;
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onUpdateGNSSMeasurementsStart(this);
            }

            mMeasurements = new ArrayList<>(measurements);

            mLsEstimator.setMeasurements(mMeasurements);
            mLsEstimator.setPriorPositionAndVelocityFromEstimation(mEstimation);
            if (mEstimation != null) {
                mLsEstimator.estimate(mEstimation);
            } else {
                mEstimation = mLsEstimator.estimate();
            }

            if (mListener != null) {
                mListener.onUpdateGNSSMeasurementsEnd(this);
            }
        } catch (final GNSSException e) {
            throw new INSGNSSException(e);
        } finally {
            mRunning = false;
        }

        if (mKinematics == null) {
            mKinematics = mInsEstimator.getKinematics();
        } else {
            mInsEstimator.getKinematics(mKinematics);
        }

        updateBodyKinematics(mKinematics, timestamp);

        return true;
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU).
     * @param timestamp  timestamp since epoch time when specific force and
     *                   angular rate values were updated.
     * @return true if body kinematics values were updated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(
            final BodyKinematics kinematics, final Time timestamp)
            throws LockedException, INSGNSSException {
        return updateBodyKinematics(kinematics, TimeConverter.convert(
                timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an intertial unit (IMU).
     * @param timestamp  timestamp expressed in seconds since epoch time when specific
     *                   force and angular rate values were updated.
     * @return true if body kinematics values were updated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(
            final BodyKinematics kinematics, final double timestamp)
            throws LockedException, INSGNSSException {

        if (mRunning) {
            throw new LockedException();
        }

        try {
            mRunning = true;

            initFrame();
            if (mEstimation != null) {
                mFrame.setCoordinates(
                        mEstimation.getX(), mEstimation.getY(), mEstimation.getZ());
                mFrame.setVelocityCoordinates(
                        mEstimation.getVx(), mEstimation.getVy(), mEstimation.getVz());
            }
            mInsEstimator.setFrame(mFrame);

            boolean result = false;
            if (mInsEstimator.isUpdateReady() &&
                    (result = mInsEstimator.update(kinematics, timestamp))) {
                if (mKinematics == null) {
                    mKinematics = mInsEstimator.getKinematics();
                } else {
                    mInsEstimator.getKinematics(mKinematics);
                }

                if (mEstimation != null) {
                    initState();
                    mInsEstimator.getState(mState);
                }
            } else {
                mKinematics = kinematics;
            }

            if (kinematics != null) {
                correctKinematics(kinematics);
            }

            return result;

        } catch (final INSException | NotReadyException e) {
            throw new INSGNSSException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Indicates whether this estimator is ready for state propagations.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return mInsEstimator.isPropagateReady() && mEstimation != null;
    }

    /**
     * Propagates Kalman filter state held by this estimator at provided
     * timestamp.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param timestamp timestamp since epoch to propagate state.
     * @return true if state was propagated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final Time timestamp) throws LockedException,
            INSGNSSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(),
                timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Propagates Kalman filter state held by this estimator at provided
     * timestamp.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param timestamp timestamp expressed in seconds since epoch to propagate state.
     * @return true if state was propagated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final double timestamp) throws LockedException,
            INSGNSSException {

        if (mRunning) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            return false;
        }

        try {
            mRunning = true;

            if (initFrame()) {
                mFrame.setCoordinates(
                        mEstimation.getX(), mEstimation.getY(), mEstimation.getZ());
                mFrame.setVelocityCoordinates(
                        mEstimation.getVx(), mEstimation.getVy(), mEstimation.getVz());
                mInsEstimator.setFrame(mFrame);
            }

            boolean result = mInsEstimator.propagate(timestamp);
            if (result) {
                mInsEstimator.getFrame(mFrame);
                mEstimation.setPositionCoordinates(
                        mFrame.getX(), mFrame.getY(), mFrame.getZ());
                mEstimation.setVelocityCoordinates(
                        mFrame.getVx(), mFrame.getVy(), mFrame.getVz());

                if (mEstimation != null) {
                    initState();
                    mInsEstimator.getState(mState);
                }
            }

            return result;

        } catch (final INSException | NotReadyException e) {
            throw new INSGNSSException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Resets this estimator.
     *
     * @throws LockedException if this estimator is already running.
     */
    public void reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;
        mMeasurements = null;
        mEstimation = null;
        mState = null;
        mKinematics = null;
        mCorrectedKinematics = null;
        mFrame = null;

        mInsEstimator.reset();

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;
    }

    /**
     * Initializes current ECEF frame containing user position, velocity and
     * orientation expressed and resolved in ECEF coordinates.
     * This method makes no action if an initial frame already exists.
     *
     * @return true if frame was initialized, false otherwise.
     */
    private boolean initFrame() {
        if (mFrame == null) {
            mFrame = new ECEFFrame();
            mFrame.setCoordinates(
                    Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Initializes state.
     */
    private void initState() {
        if (mState == null) {
            mState = new INSLooselyCoupledKalmanState();
        }
    }

    /**
     * Corrects provided kinematics by taking into account currently estimated
     * specific force and angular rate biases.
     * This method stores the result into the variable member containing corrected
     * kinematics values.
     *
     * @param kinematics kinematics instance to be corrected.
     */
    private void correctKinematics(final BodyKinematics kinematics) {
        if (mCorrectedKinematics == null) {
            mCorrectedKinematics = new BodyKinematics();
        }

        final double accelBiasX;
        final double accelBiasY;
        final double accelBiasZ;
        final double gyroBiasX;
        final double gyroBiasY;
        final double gyroBiasZ;
        if (mState != null) {
            accelBiasX = getValueOrZero(mState.getAccelerationBiasX());
            accelBiasY = getValueOrZero(mState.getAccelerationBiasY());
            accelBiasZ = getValueOrZero(mState.getAccelerationBiasZ());
            gyroBiasX = getValueOrZero(mState.getGyroBiasX());
            gyroBiasY = getValueOrZero(mState.getGyroBiasY());
            gyroBiasZ = getValueOrZero(mState.getGyroBiasZ());
        } else {
            accelBiasX = 0.0;
            accelBiasY = 0.0;
            accelBiasZ = 0.0;
            gyroBiasX = 0.0;
            gyroBiasY = 0.0;
            gyroBiasZ = 0.0;
        }

        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();
        final double angularRateX = kinematics.getAngularRateX();
        final double angularRateY = kinematics.getAngularRateY();
        final double angularRateZ = kinematics.getAngularRateZ();

        mCorrectedKinematics.setSpecificForceCoordinates(
                fx - accelBiasX, fy - accelBiasY, fz - accelBiasZ);
        mCorrectedKinematics.setAngularRateCoordinates(
                angularRateX - gyroBiasX,
                angularRateY - gyroBiasY,
                angularRateZ - gyroBiasZ);
    }

    /**
     * Returns provided value if not infinity and not NaN.
     *
     * @param value value to be returned.
     * @return value or 0.0.
     */
    private double getValueOrZero(final double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return 0.0;
        } else {
            return value;
        }
    }
}
