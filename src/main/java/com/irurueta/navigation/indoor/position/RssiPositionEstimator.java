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
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Base class for position estimators based on located fingerprints containing only
 * RSSI readings and having as well prior knowledge of the location of radio sources
 * associated to those readings.
 * This implementation uses a first-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class RssiPositionEstimator<P extends Point> extends
        BaseRssiPositionEstimator<P, SourcedRssiPositionEstimatorListener<P>> {

    /**
     * Located radio sources.
     */
    protected List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     */
    protected boolean mUseSourcesPathLossExponentWhenAvailable = true;

    /**
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     */
    protected boolean mUseNoMeanNearestFingerprintFinder = true;

    /**
     * True indicates that mean effects are removed from located fingerprints and from
     * new fingerprints whose location is unknown.
     * By default this is disabled to remove possible bias effects due to readings
     * measured by different devices with different hardware.
     */
    protected boolean mRemoveMeansFromFingerprintReadings;

    /**
     * Constructor.
     */
    public RssiPositionEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public RssiPositionEstimator(
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public RssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources) {
        super(locatedFingerprints, fingerprint);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public RssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        internalSetSources(sources);
    }

    /**
     * Gets located radio sources.
     * @return located radio sources.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Sets located radio sources.
     * @param sources located radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setSources(List<? extends RadioSourceLocated<P>> sources)
            throws LockedException  {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     * @return true to use path loss exponent of provided sources when available,
     * false otherwise.
     */
    public boolean getUseSourcesPathLossExponentWhenAvailable() {
        return mUseSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Specifies whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseSourcesPathLossExponentWhenAvailable(
            boolean useSourcesPathLossExponentWhenAvailable) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Indicates which fingerprint finder is used.
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     * @return indicates which fingerprint finder is used.
     */
    public boolean getUseNoMeanNearestFingerprintFinder() {
        return mUseNoMeanNearestFingerprintFinder;
    }

    /**
     * Specifies which fingerprint finder is used.
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     * @param useNoMeanNearestFingerprintFinder indicates which fingerprint finder is used.
     * @throws LockedException if estimator is locked.
     */
    public void setUseNoMeanNearestFingerprintFinder(
            boolean useNoMeanNearestFingerprintFinder) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseNoMeanNearestFingerprintFinder = useNoMeanNearestFingerprintFinder;
    }

    /**
     * Indicates whether mean effects are removed from fingerprints.
     * True indicates that mean effects are removed from located fingerprints and from
     * new fingerprints whose location is unknown.
     * By default this is enabled to remove possible bias effects due to readings
     * measured by different devices with different hardware.
     * @return true to remove mean effects, false otherwise.
     */
    public boolean isMeansFromFingerprintReadingsRemoved() {
        return mRemoveMeansFromFingerprintReadings;
    }

    /**
     * Specifies whether mean effects are removed from fingerprints.
     * True indicates that mean effects are removed from located fingerprints and from
     * new fingerprints whose location is unknown.
     * By default this is enabled to remove possible bias effects due to readings
     * measured by different devices with different hardware.
     * @param removeMeansFromFingerprintReadings true to remove mean effects, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setMeansFromFingerprintReadingsRemoved(
            boolean removeMeansFromFingerprintReadings) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRemoveMeansFromFingerprintReadings = removeMeansFromFingerprintReadings;
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mSources != null && mLocatedFingerprints != null &&
                mFingerprint != null;
    }

    /**
     * Internally sets located radio sources.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetSources(List<? extends RadioSourceLocated<P>> sources) {
        if (sources == null) {
            throw new IllegalArgumentException();
        }

        mSources = sources;
    }
}
