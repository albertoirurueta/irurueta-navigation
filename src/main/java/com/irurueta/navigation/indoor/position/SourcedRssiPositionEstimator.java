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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;

import java.util.Collection;
import java.util.List;

/**
 * Base class for position estimators based on located fingerprints containing only
 * RSSI readings and having as well prior knowledge of the location of radio sources
 * associated to those readings.
 * @param <P> a {@link Point} type.
 */
public abstract class SourcedRssiPositionEstimator<P extends Point> extends
        BaseRssiPositionEstimator<P, SourcedRssiPositionEstimatorListener<P>> {

    /**
     * Located radio sources.
     */
    private List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Indicates whether pathloss exponent of provided sources must be used when
     * available (if true), or if fallback pathloss exponent must be used instead.
     */
    private boolean mUseSourcesPathLossExponentWhenAvailable = true;

    /**
     * Constructor.
     */
    public SourcedRssiPositionEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator(
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints) {
        super(locatedFingerprints);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super(fingerprint);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, listener);
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
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super(locatedFingerprints, fingerprint);
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
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints) {
        super(minNearestFingerprints, maxNearestFingerprints);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(minNearestFingerprints, maxNearestFingerprints, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, listener);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     */
    public SourcedRssiPositionEstimator(double pathLossExponent) {
        super(pathLossExponent);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator(double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, pathLossExponent);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent) {
        super(fingerprint, pathLossExponent);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint, pathLossExponent);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources) {
        super(locatedFingerprints);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources) {
        super(fingerprint);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, listener);
        internalSetSources(sources);
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
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
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
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, minNearestFingerprints,
                maxNearestFingerprints, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources, double pathLossExponent) {
        super(locatedFingerprints, pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent) {
        super(fingerprint, pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint, pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent, listener);
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator(
            boolean useSourcesPathLossExponentWhenAvailable) {
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator(
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(minNearestFingerprints, maxNearestFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator(double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator(double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     * long as it has a limit defined), or if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (as long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, sources);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, sources, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(fingerprint, sources);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(fingerprint, sources, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, fingerprint, sources);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, sources, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources, double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, sources, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, sources, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        this(fingerprint, sources, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(fingerprint, sources, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, fingerprint, sources, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, sources, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     * fingerprints or readings within provided fingerprints (for 2D position estimation at
     * least 2 located total readings are required among all fingerprints, for example
     * 2 readings are required in a single fingerprint, or at least 2 fingerprints
     * at different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints), or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided fingerprint value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        this(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        this(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints) or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent, listener);
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
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
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mSources != null && mLocatedFingerprints != null &&
                mFingerprint != null;
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    public void estimate() throws LockedException, NotReadyException,
            PositionEstimationException {

        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            //noinspection unchecked
            RadioSourceNoMeanKNearestFinder<P, RadioSource> finder =
                    new RadioSourceNoMeanKNearestFinder<>((Collection<? extends RssiFingerprintLocated<RadioSource,
                            RssiReading<RadioSource>, P>>)mLocatedFingerprints);

            mEstimatedPositionCoordinates = null;

            int dims = getNumberOfDimensions();
            int max = mMaxNearestFingerprints < 0 ?
                    mLocatedFingerprints.size() : mMaxNearestFingerprints;
            for (int k = mMinNearestFingerprints; k < max; k++) {
                //noinspection unchecked
                List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>>
                        nearestFingerprints = finder.findKNearestTo(
                        (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) mFingerprint, k);

                //build system of equations
                int totalReadings = totalReadings(nearestFingerprints);

                try {
                    double ln10 = Math.log(10.0);
                    int row = 0;
                    Matrix a = new Matrix(totalReadings, dims);
                    double[] b = new double[totalReadings];
                    for (RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P> locatedFingerprint :
                            nearestFingerprints) {

                        P fingerprintPosition = locatedFingerprint.getPosition();
                        List<RssiReading<RadioSource>> locatedReadings =
                                locatedFingerprint.getReadings();
                        if (locatedReadings == null) {
                            continue;
                        }


                        for (RssiReading<RadioSource> locatedReading : locatedReadings) {
                            RadioSource source = locatedReading.getSource();

                            //find within the list of located sources the source of
                            //current located fingerprint reading
                            int pos = mSources.indexOf(source);
                            if (pos < 0) {
                                continue;
                            }

                            RadioSourceLocated<P> locatedSource = mSources.get(pos);
                            double pathLossExponent = mPathLossExponent;
                            if (mUseSourcesPathLossExponentWhenAvailable &&
                                    locatedSource instanceof RadioSourceWithPower) {
                                pathLossExponent = ((RadioSourceWithPower)locatedFingerprint).
                                        getPathLossExponent();
                            }

                            double tmp = 10.0 * pathLossExponent / ln10;

                            P sourcePosition = locatedSource.getPosition();
                            double locatedRssi = locatedReading.getRssi();
                            double sqrDistance = fingerprintPosition.sqrDistanceTo(sourcePosition);

                            List<? extends RssiReading<? extends RadioSource>> readings =
                                    mFingerprint.getReadings();
                            for (RssiReading<? extends RadioSource> reading : readings) {
                                double rssi = reading.getRssi();
                                double diffRssi = locatedRssi - rssi;

                                b[row] = -diffRssi;
                                for (int i = 0; i < dims; i++) {
                                    double fingerprintCoord = fingerprintPosition.getInhomogeneousCoordinate(i);
                                    double sourceCoord = sourcePosition.getInhomogeneousCoordinate(i);
                                    double diffCoord = fingerprintCoord - sourceCoord;

                                    a.setElementAt(row, i, tmp * diffCoord / sqrDistance);

                                    b[row] += tmp * diffCoord / sqrDistance * fingerprintCoord;
                                }
                                row++;
                            }
                        }
                    }

                    mEstimatedPositionCoordinates = com.irurueta.algebra.Utils.solve(a, b);

                    //a solution was found so we exit loop
                    break;
                } catch (AlgebraException e) {
                    //solution could not be found with current data
                    //Iterate to use additional nearby fingerprints
                    mEstimatedPositionCoordinates = null;
                }
            }

            if (mEstimatedPositionCoordinates == null) {
                //no solution could be found
                throw new PositionEstimationException();
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } finally {
            mLocked = false;
        }
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
