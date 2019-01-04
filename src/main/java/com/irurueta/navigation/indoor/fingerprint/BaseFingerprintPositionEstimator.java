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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiFingerprintLocated;
import com.irurueta.navigation.indoor.RssiReading;

import java.util.List;

/**
 * Base class for position estimators based on located fingerprints containing only
 * RSSI readings without any prior knowledge of radio sources.
 * These kind of estimators can be used to determine the position of a given device by
 * getting RSSI readings at an unknown location of different radio sources and comparing
 * those readings with other located ones.
 * @param <P> a {@link Point} type.
 * @param <L> a {@link BaseFingerprintEstimatorListener} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class BaseFingerprintPositionEstimator<P extends Point,
        L extends BaseFingerprintEstimatorListener> {

    /**
     * Default minimum number of nearest fingerprints to search.
     */
    public static final int DEFAULT_MIN_NEAREST_FINGERPRINTS = 1;

    /**
     * Default maximum number of nearest fingerprints to search (no limit).
     */
    public static final int DEFAULT_MAX_NEAREST_FINGERPRINTS = -1;

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Located fingerprints containing RSSI readings.
     */
    protected List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> mLocatedFingerprints;

    /**
     * Fingerprint containing readings at an unknown location.
     * Readings need to belong to the same radio sources as those given at located
     * fingerprints
     */
    protected RssiFingerprint<? extends RadioSource,
                ? extends RssiReading<? extends RadioSource>> mFingerprint;

    /**
     * Minimum number of nearest fingerprints to search.
     */
    protected int mMinNearestFingerprints = DEFAULT_MIN_NEAREST_FINGERPRINTS;

    /**
     * Maximum number of nearest fingerprints to search or -1 if there is no limit and
     * all provided fingerprints are used.
     */
    protected int mMaxNearestFingerprints = DEFAULT_MAX_NEAREST_FINGERPRINTS;

    /**
     * Path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     */
    protected double mPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected L mListener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] mEstimatedPositionCoordinates;

    /**
     * Nearest located fingerprints based on their RSSI readings respect to provided fingerprint.
     * These are the fingerprints that are probably located close to the unknown location to be estimated,
     * however their location is approximate due to errors on RSSI readings.
     */
    protected List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> mNearestFingerprints;

    /**
     * Indicates if this instance is locked.
     */
    protected boolean mLocked;

    /**
     * Constructor.
     */
    public BaseFingerprintPositionEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public BaseFingerprintPositionEstimator(L listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public BaseFingerprintPositionEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint) {
        internalSetLocatedFingerprints(locatedFingerprints);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public BaseFingerprintPositionEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            L listener) {
        this(listener);
        internalSetLocatedFingerprints(locatedFingerprints);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets located fingerprints containing RSSI readings.
     * @return located fingerprints containing RSSI readings.
     */
    public List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> getLocatedFingerprints() {
        return mLocatedFingerprints;
    }

    /**
     * Sets located fingerprints containing RSSI readings.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 reading in a single fingerprint, or 3 fingerprints containing a single
     * reading or any combination resulting in at least 3 readings at different locations
     * are required).
     */
    public void setLocatedFingerprints(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetLocatedFingerprints(locatedFingerprints);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located
     * fingerprints.
     * @return fingerprint containing readings at an unknown location for provided located
     * fingerprints.
     */
    public RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>>
            getFingerprint() {
        return mFingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided located fingerprints.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located fingerprints.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is null.
     */
    public void setFingerprint(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Get minimum number of nearest fingerprints to search.
     * @return minimum number of nearest fingerprints.
     */
    public int getMinNearestFingerprints() {
        return mMinNearestFingerprints;
    }

    /**
     * Gets maximum number of nearest fingerprints to search.
     * @return maximum number of nearest fingerprints.
     */
    public int getMaxNearestFingerprints() {
        return mMaxNearestFingerprints;
    }

    /**
     * Sets minimum and maximum number of nearest fingerprints to search.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    public void setMinMaxNearestFingerprints(int minNearestFingerprints,
            int maxNearestFingerprints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetMinMaxNearestFingerprints(minNearestFingerprints,
                maxNearestFingerprints);
    }

    /**
     * Gets path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * @return path loss exponent to be used by default.
     */
    public double getPathLossExponent() {
        return mPathLossExponent;
    }

    /**
     * Sets path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws LockedException if estimator is locked.
     */
    public void setPathLossExponent(double pathLossExponent) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPathLossExponent = pathLossExponent;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     * @return listener to be notified of events raised by this instance.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(L listener) throws LockedException {
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
     * Gets nearest found located fingerprints based on their RSSI readings respect to provided fingerprint.
     * These are the fingerprints that are probably located close to the unknown location to be estimated,
     * however their location is approximate due to errors on RSSI readings.
     * @return nearest located fingerprints based on their RSSI readings or null if estimation has not been done yet.
     */
    public List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> getNearestFingerprints() {
        return mNearestFingerprints;
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation
     * is already in progress.
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Gets number of dimensions of points.
     * @return number of dimensions of points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Starts estimation based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws FingerprintEstimationException if estimation fails for some other reason.
     */
    public abstract void estimate() throws LockedException, NotReadyException, FingerprintEstimationException;

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets total number of readings contained within provided fingerprints.
     * @param locatedFingerprints fingerprints to be checked.
     * @return total number of readings contained within provided fingerprints.
     */
    int totalReadings(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints) {
        if (locatedFingerprints == null) {
            return 0;
        }

        int totalReadings = 0;
        for (RssiFingerprintLocated<? extends RadioSource,
                ? extends RssiReading<? extends RadioSource>, P> fingerprint : locatedFingerprints) {
            List<? extends RssiReading<? extends RadioSource>> readings =
                    fingerprint.getReadings();
            if (readings != null) {
                totalReadings += readings.size();
            }
        }
        return totalReadings;
    }

    /**
     * Internally sets located fingerprints containing RSSI readings.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    private void internalSetLocatedFingerprints(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints) {

        if (totalReadings(locatedFingerprints) < getNumberOfDimensions()) {
            throw new IllegalArgumentException();
        }

        mLocatedFingerprints = locatedFingerprints;
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided located fingerprints.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located fingerprints.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetFingerprint(
            RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        mFingerprint = fingerprint;
    }

    /**
     * Sets minimum and maximum number of nearest fingerprints to search.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    @SuppressWarnings("Duplicates")
    private void internalSetMinMaxNearestFingerprints(int minNearestFingerprints,
            int maxNearestFingerprints) {
        if (minNearestFingerprints < 1 ||
                (maxNearestFingerprints >= 0 && minNearestFingerprints > maxNearestFingerprints)) {
            throw new IllegalArgumentException();
        }

        mMinNearestFingerprints = minNearestFingerprints;
        mMaxNearestFingerprints = maxNearestFingerprints;
    }
}
