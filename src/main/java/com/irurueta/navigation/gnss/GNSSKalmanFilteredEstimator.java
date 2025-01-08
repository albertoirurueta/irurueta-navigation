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
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_Kalman_Filter.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_Kalman_Filter.m
 * </a>
 */
public class GNSSKalmanFilteredEstimator {

    /**
     * Internal estimator to compute least squares solution for GNSS measurements.
     */
    private final GNSSLeastSquaresPositionAndVelocityEstimator lsEstimator =
            new GNSSLeastSquaresPositionAndVelocityEstimator();

    /**
     * Listener to notify events raised by this instance.
     */
    private GNSSKalmanFilteredEstimatorListener listener;

    /**
     * Minimum epoch interval expressed in seconds (s) between consecutive
     * propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements when
     * intervals are lass than this value, will be ignored.
     */
    private double epochInterval;

    /**
     * GNSS Kalman filter configuration parameters (usually obtained through calibration).
     */
    private GNSSKalmanConfig config;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> measurements;

    /**
     * Current estimation containing user ECEF position, user ECEF velocity, clock offset
     * and clock drift.
     */
    private GNSSEstimation estimation;

    /**
     * Current Kalman filter state containing current GNSS estimation along with
     * Kalman filter covariance error matrix.
     */
    private GNSSKalmanState state;

    /**
     * Timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated.
     */
    private Double lastStateTimestamp;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean running;

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
        this.config = new GNSSKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
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
    public GNSSKalmanFilteredEstimator(final GNSSKalmanFilteredEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final GNSSKalmanConfig config, final double epochInterval) {
        this(epochInterval);
        this.config = new GNSSKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param config   GNSS Kalman filter configuration parameters (usually
     *                 obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanConfig config, final GNSSKalmanFilteredEstimatorListener listener) {
        this(config);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(
            final double epochInterval, final GNSSKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanConfig config, final double epochInterval,
            final GNSSKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final Time epochInterval) {
        this(TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final GNSSKalmanConfig config, final Time epochInterval) {
        this(config, TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(final Time epochInterval, final GNSSKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        GNSS Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public GNSSKalmanFilteredEstimator(
            final GNSSKalmanConfig config, final Time epochInterval,
            final GNSSKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        this.listener = listener;
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public GNSSKalmanFilteredEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(final GNSSKalmanFilteredEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
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
        return epochInterval;
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
        if (running) {
            throw new LockedException();
        }

        if (epochInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        this.epochInterval = epochInterval;
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param result instance where minimum epoch interval will be stored.
     */
    public void getEpochIntervalAsTime(final Time result) {
        result.setValue(epochInterval);
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
        return new Time(epochInterval, TimeUnit.SECOND);
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
        final var epochIntervalSeconds = TimeConverter.convert(epochInterval.getValue().doubleValue(),
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
        if (config != null) {
            result.copyFrom(config);
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
        return config;
    }

    /**
     * Sets GNSS Kalman configuration parameters (usually obtained through
     * calibration).
     *
     * @param config GNSS Kalman configuration parameters to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setConfig(final GNSSKalmanConfig config) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.config = new GNSSKalmanConfig(config);
    }

    /**
     * Gets last updated GNSS measurements of a collection of satellites.
     *
     * @return last updated GNSS measurements of a collection of satellites.
     */
    public Collection<GNSSMeasurement> getMeasurements() {
        if (measurements == null) {
            return null;
        }

        final var result = new ArrayList<GNSSMeasurement>();
        for (final var measurement : measurements) {
            result.add(new GNSSMeasurement(measurement));
        }
        return result;
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     *
     * @return current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     */
    public GNSSEstimation getEstimation() {
        return estimation != null ? new GNSSEstimation(estimation) : null;
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
        if (estimation != null) {
            result.copyFrom(estimation);
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
        return state != null ? new GNSSKalmanState(state) : null;
    }

    /**
     * Gets current Kalman filter state containing current GNSS estimation along with
     * Kalman filter covariance error matrix.
     * This method does not update result instance if no state is available.
     *
     * @param result instance where state will be stored.
     * @return true if result state was updated, false otherwise.
     */
    public boolean getState(final GNSSKalmanState result) {
        if (state != null) {
            result.copyFrom(state);
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
        return lastStateTimestamp;
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @param result instance where timestamp since epoch time when Kalman filter
     *               state was last propagated will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastStateTimestampAsTime(final Time result) {
        if (lastStateTimestamp != null) {
            result.setValue(lastStateTimestamp);
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
        return lastStateTimestamp != null ? new Time(lastStateTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Indicates whether provided measurements are ready to
     * be used for an update.
     *
     * @param measurements measurements to be checked.
     * @return true if estimator is ready, false otherwise.
     */
    public static boolean isUpdateMeasurementsReady(final Collection<GNSSMeasurement> measurements) {
        return GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(measurements);
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

        if (running) {
            throw new LockedException();
        }

        if (!isUpdateMeasurementsReady(measurements)) {
            throw new NotReadyException();
        }

        if (lastStateTimestamp != null && timestamp - lastStateTimestamp <= epochInterval) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateStart(this);
            }

            this.measurements = new ArrayList<>(measurements);

            lsEstimator.setMeasurements(this.measurements);
            lsEstimator.setPriorPositionAndVelocityFromEstimation(estimation);
            if (estimation != null) {
                lsEstimator.estimate(estimation);
            } else {
                estimation = lsEstimator.estimate();
            }

            if (listener != null) {
                listener.onUpdateEnd(this);
            }

        } finally {
            running = false;
        }

        propagate(timestamp);

        return true;
    }

    /**
     * Indicates whether this estimator is ready for state propagations.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return config != null && estimation != null;
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
    public boolean propagate(final Time timestamp) throws LockedException, NotReadyException, GNSSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
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
    public boolean propagate(final double timestamp) throws LockedException, NotReadyException, GNSSException {

        if (running) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            throw new NotReadyException();
        }

        final var propagationInterval = lastStateTimestamp != null ? timestamp - lastStateTimestamp : 0.0;
        if (lastStateTimestamp != null && propagationInterval <= epochInterval) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onPropagateStart(this);
            }

            if (state == null) {
                state = GNSSKalmanInitializer.initialize(estimation, config);
            }

            GNSSKalmanEpochEstimator.estimate(measurements, propagationInterval, state, config, state);
            lastStateTimestamp = timestamp;

            state.getEstimation(estimation);

            if (listener != null) {
                listener.onPropagateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new GNSSException(e);
        } finally {
            running = false;
        }

        return true;
    }

    /**
     * Resets this estimator.
     *
     * @throws LockedException if this estimator is already running.
     */
    public void reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;
        measurements = null;
        estimation = null;
        state = null;
        lastStateTimestamp = null;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }
}
