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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

@SuppressWarnings("WeakerAccess")
public class SourcedRssiPositionEstimator2D extends SourcedRssiPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public SourcedRssiPositionEstimator2D() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator2D(
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints) {
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
    public SourcedRssiPositionEstimator2D(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     */
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
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
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, listener);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     */
    public SourcedRssiPositionEstimator2D(double pathLossExponent) {
        super(pathLossExponent);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator2D(double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param pathLossExponent path loss exponent to be used by default.
     */
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
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
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources) {
        super(locatedFingerprints, sources);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources) {
        super(fingerprint, sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources) {
        super(locatedFingerprints, fingerprint, sources);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, sources,
                minNearestFingerprints, maxNearestFingerprints);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources,
                minNearestFingerprints, maxNearestFingerprints, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints) {
        super(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent) {
        super(locatedFingerprints, sources, pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws IllegalArgumentException if provided value is null.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent) {
        super(fingerprint, sources, pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, pathLossExponent, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint, sources, pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, pathLossExponent, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent) {
        super(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator2D(
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(useSourcesPathLossExponentWhenAvailable);
    }

    /**
     * Constructor.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator2D(
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint,
                useSourcesPathLossExponentWhenAvailable, listener);
    }

    /**
     * Constructor.
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable, listener);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     */
    public SourcedRssiPositionEstimator2D(double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(pathLossExponent, useSourcesPathLossExponentWhenAvailable);
    }

    /**
     * Constructor.
     * @param pathLossExponent path loss exponent to be used by default.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @param listener listener in charge of handling events.
     */
    public SourcedRssiPositionEstimator2D(double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(pathLossExponent, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(int minNearestFingerprints,
            int maxNearestFingerprints, double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint,
                minNearestFingerprints, maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, sources, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, useSourcesPathLossExponentWhenAvailable,
                listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, sources, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, useSourcesPathLossExponentWhenAvailable,
                listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, sources,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, useSourcesPathLossExponentWhenAvailable,
                listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided value is null or if
     * minimum value is larger than maximum value (As long as it has a limit defined), or
     * if minimum value is less than 1.
     */
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, useSourcesPathLossExponentWhenAvailable,
                listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent,
            boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, sources, minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable, listener);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable) {
        super(locatedFingerprints, fingerprint, sources,
                minNearestFingerprints, maxNearestFingerprints,
                pathLossExponent, useSourcesPathLossExponentWhenAvailable);
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
    public SourcedRssiPositionEstimator2D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point2D>> sources,
            int minNearestFingerprints, int maxNearestFingerprints,
            double pathLossExponent, boolean useSourcesPathLossExponentWhenAvailable,
            SourcedRssiPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, minNearestFingerprints,
                maxNearestFingerprints, pathLossExponent,
                useSourcesPathLossExponentWhenAvailable, listener);
    }

    /**
     * Gets number of dimensions of points.
     * @return number of dimensions of points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        Point2D result = new InhomogeneousPoint2D();
        getEstimatedPosition(result);
        return result;
    }
}
