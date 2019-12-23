/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.gnss;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Calculates position, velocity, clock offset and clock drift using an
 * unweighted iterated least squares estimator along with a Kalman filter
 * to smooth results.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_Kalman_Filter.m
 */
public class GNSSKalmanFilteredEstimator {

    /**
     * Internal estimator to compute least squares solution for GNSS measurements.
     */
    private GNSSLeastSquaresPositionAndVelocityEstimator mLsEstimator =
            new GNSSLeastSquaresPositionAndVelocityEstimator();

    /**
     * Listener to notify events raised by this instance.
     */
    private GNSSKalmanFilteredEstimatorListener mListener;

    /**
     * Minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements when
     * intervals are lass than this value, will be ignored.
     */
    private double mEpochInterval;

    /**
     * GNSS Kalman filter configuration parameters (usually obtained through calibration).
     */
    private GNSSKalmanConfig mConfig;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> mMeasurements;

    /**
     * Current estimation containing user ECEF position, user ECEF velocity, clock offset
     * and clock drift.
     */
    private GNSSEstimation mEstimation;

    /**
     * Current Kalman filter state containing current GNSS estimation along with
     * Kalman filter covariance error matrix.
     */
    private GNSSKalmanState mState;

    /**
     * Timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated.
     */
    private Double mLastStateTimestamp;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public GNSSKalmanFilteredEstimator() {
    }

    /**
     * Constructor.
     *
     * @param config GNSS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     */
    public GNSSKalmanFilteredEstimator(final GNSSKalmanConfig config) {
        mConfig = new GNSSKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations or
     *                      measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final double epochInterval) {
        try {
            setEpochInterval(epochInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to notify events raised by this instance.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanFilteredEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations or
     *                      measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final GNSSKalmanConfig config,
                                       final double epochInterval) {
        this(epochInterval);
        mConfig = new GNSSKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param config   GNSS Kalman filter configuration parameters (usually
     *                 obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanConfig config,
            final GNSSKalmanFilteredEstimatorListener listener) {
        this(config);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations or
     *                      measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(
            final double epochInterval,
            final GNSSKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations or
     *                      measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanConfig config, final double epochInterval,
            final GNSSKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        mListener = listener;
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public GNSSKalmanFilteredEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(final GNSSKalmanFilteredEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements
     * expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval between consecutive propagations or
     * measurements.
     */
    public double getEpochInterval() {
        return mEpochInterval;
    }

    /**
     * Sets minimum epoch interval between consecutive propagations or measurements
     * expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public void setEpochInterval(final double epochInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (epochInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        mEpochInterval = epochInterval;
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param result instance where minimum epoch interval will be stored.
     */
    public void getEpochIntervalAsTime(final Time result) {
        result.setValue(mEpochInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval.
     */
    public Time getEpochIntervalAsTime() {
        return new Time(mEpochInterval, TimeUnit.SECOND);
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
        final double epochIntervalSeconds = TimeConverter.convert(
                epochInterval.getValue().doubleValue(),
                epochInterval.getUnit(), TimeUnit.SECOND);
        setEpochInterval(epochIntervalSeconds);
    }

    /**
     * Gets GNSS Kalman configuration parameters (usually obtained through
     * calibration).
     *
     * @param result instance where GNSS Kalman configuration parameters will be
     *               stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getConfig(final GNSSKalmanConfig result) {
        if (mConfig != null) {
            result.copyFrom(mConfig);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets GNSS Kalman configuration parameters (usually obtained through
     * calibration).
     *
     * @return GNSS Kalman configuration parameters.
     */
    public GNSSKalmanConfig getConfig() {
        return mConfig;
    }

    /**
     * Sets GNSS Kalman configuration parameters (usually obtained through
     * calibration).
     *
     * @param config GNSS Kalman configuration parameters to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setConfig(final GNSSKalmanConfig config)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mConfig = new GNSSKalmanConfig(config);
    }

    /**
     * Gets last updated GNSS measurements of a collection of satellites.
     *
     * @return last updated GNSS measurements of a collection of satellites.
     */
    public Collection<GNSSMeasurement> getMeasurements() {
        return mMeasurements != null ?
                new ArrayList<>(mMeasurements) : null;
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
     * Gets current Kalman filter state containing current GNSS estimation along with
     * Kalman filter covariance error matrix.
     *
     * @return current Kalman filter state containing current GNSS estimation along
     * with Kalman filter covariance error matrix.
     */
    public GNSSKalmanState getState() {
        return mState != null ? new GNSSKalmanState(mState) : null;
    }

    /**
     * Gets current Kalman filter state containing current GNSS estimatino along with
     * Kalman filter covariance error matrix.
     * This method does not update result instance if no state is available.
     *
     * @param result instance where state will be stored.
     * @return true if result state was updated, false otherwise.
     */
    public boolean getState(final GNSSKalmanState result) {
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
        return mLastStateTimestamp;
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propageted.
     *
     * @param result instance where timestamp since epoch time when Kalman filter
     *               state was last propagated will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastStateTimestampAsTime(final Time result) {
        if (mLastStateTimestamp != null) {
            result.setValue(mLastStateTimestamp);
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @return timestamp since epoch time when Kalman filter state was last
     * propagated.
     */
    public Time getLastStateTimestampAsTime() {
        return mLastStateTimestamp != null ?
                new Time(mLastStateTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether provided measurements are ready to
     * be used for an update.
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
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp since epoch time when GNSS measurements were
     *                     updated.
     * @return true if measurements were updated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(
            final Collection<GNSSMeasurement> measurements, final Time timestamp)
            throws LockedException, NotReadyException, GNSSException {
        return updateMeasurements(measurements, TimeConverter.convert(
                timestamp.getValue().doubleValue(), timestamp.getUnit(), TimeUnit.SECOND));
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
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(
            final Collection<GNSSMeasurement> measurements, final double timestamp)
            throws LockedException, NotReadyException, GNSSException {

        if (mRunning) {
            throw new LockedException();
        }

        if (!isUpdateMeasurementsReady(measurements)) {
            throw new NotReadyException();
        }

        if (mLastStateTimestamp != null &&
                timestamp - mLastStateTimestamp <= mEpochInterval) {
            return false;
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onUpdateStart(this);
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
                mListener.onUpdateEnd(this);
            }

            mRunning = false;

            propagate(timestamp);
        } finally {
            mRunning = false;
        }

        return true;
    }

    /**
     * Indicates whether this estimator is ready for state propagations.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return mConfig != null && mEstimation != null;
    }

    /**
     * Propagates Kalman filter state held by this estimator at provided
     * timestamp.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param timestamp timestamp since epoch to propagate state.
     * @return true if state was propagated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final Time timestamp) throws LockedException,
            NotReadyException, GNSSException {
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
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final double timestamp) throws LockedException,
            NotReadyException, GNSSException {

        if (mRunning) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            throw new NotReadyException();
        }

        final double propagationInterval = mLastStateTimestamp != null ?
                timestamp - mLastStateTimestamp : 0.0;
        if (mLastStateTimestamp != null && propagationInterval <= mEpochInterval) {
            return false;
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onPropagateStart(this);
            }

            if (mState == null) {
                mState = GNSSKalmanInitializer.initialize(mEstimation, mConfig);
            }

            GNSSKalmanEpochEstimator.estimate(mMeasurements, propagationInterval,
                    mState, mConfig, mState);
            mLastStateTimestamp = timestamp;

            mState.getEstimation(mEstimation);

            if (mListener != null) {
                mListener.onPropagateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new GNSSException(e);
        } finally {
            mRunning = false;
        }

        return true;
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
        mLastStateTimestamp = null;

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;
    }
}
