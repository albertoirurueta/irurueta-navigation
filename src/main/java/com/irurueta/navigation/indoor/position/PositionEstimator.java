/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;

import java.util.List;

/**
 * Base class for position estimators using located radio sources and their readings at unknown locations.
 * These kind of estimators can be used to determine the position of a given device by getting readings at
 * an unknown location of different radio sources whose locations are known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class PositionEstimator<P extends Point> {

    /**
     * Located radio sources used for trilateration.
     */
    protected List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Fingerprint containing readings at an unknown location for provided located radio sources.
     */
    protected Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> mFingerprint;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected PositionEstimatorListener<P> mListener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] mEstimatedPositionCoordinates;

    /**
     * Constructor.
     */
    public PositionEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public PositionEstimator(PositionEstimatorListener<P> listener) {
        mListener = listener;
    }

    /**
     * Gets located radio sources used for trilateration.
     * @return located radio sources used for trilateration.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    public void setSources(List<? extends RadioSourceLocated<P>> sources)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located radio sources.
     * @return fingerprint containing readings at an unknown location for provided located radio sources.
     */
    public Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> getFingerprint() {
        return mFingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is null.
     */
    public void setFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     * @return listener to be notified of events raised by this instance.
     */
    public PositionEstimatorListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(PositionEstimatorListener<P> listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return mEstimatedPositionCoordinates;
    }

    /**
     * Gets estimated estimated position and stores result into provided instance.
     * @param estimatedPosition instance where estimated estimated position will be stored.
     */
    public void getEstimatedPosition(P estimatedPosition) {
        if (mEstimatedPositionCoordinates != null) {
            for (int i = 0; i < mEstimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i,
                        mEstimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     * @return minimum required number of located radio sources to perform trilateration.
     */
    public abstract int getMinRequiredSources();

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation is already in progress.
     * @return true if estimator is locked, false otherwise.
     */
    public abstract boolean isLocked();

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    public abstract void estimate() throws LockedException, NotReadyException, PositionEstimationException;

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets known positions of radio sources used internally to solve trilateration.
     * @return known positions used internally.
     */
    public abstract P[] getPositions();

    /**
     * Gets euclidean distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance values are used internally to solve trilateration.
     * @return euclidean distances used internally.
     */
    public abstract double[] getDistances();

    /**
     * Internally sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    @SuppressWarnings("Duplicates")
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources) {
        if (sources == null) {
            throw new IllegalArgumentException();
        }

        if (sources.size() < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mSources = sources;
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        mFingerprint = fingerprint;
    }
}
