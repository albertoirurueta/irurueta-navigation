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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * 2D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This is a base implementation for all implementations using different orders of
 * Taylor approximation to estimate position.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class NonLinearFingerprintPositionEstimator2D extends
        NonLinearFingerprintPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public NonLinearFingerprintPositionEstimator2D() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public NonLinearFingerprintPositionEstimator2D(
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public NonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super(locatedFingerprints, fingerprint, sources);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public NonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public NonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition) {
        super(locatedFingerprints, fingerprint, sources, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public NonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, initialPosition, listener);
    }

    /**
     * Gets number of dimensions of points.
     *
     * @return number of dimensions of points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated position or null if not available yet.
     *
     * @return estimated position or null.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final Point2D result = new InhomogeneousPoint2D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param type type to be used.
     * @return a non-linear 2D position estimator.
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D();
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D();
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D();
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param listener listener in charge of handling events.
     * @param type     a non-linear 2D position estimator.
     * @return a non-linear 2D position estimator.
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final FingerprintPositionEstimatorListener<Point2D> listener,
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D(listener);
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D(listener);
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D(listener);
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param type                a non-linear 2D position estimator.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources);
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources);
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources);
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param listener            listener in charge of handling events.
     * @param type                a non-linear 2D position estimator.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final FingerprintPositionEstimatorListener<Point2D> listener,
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, listener);
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, listener);
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, listener);
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @param type                a non-linear 2D position estimator.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Point2D initialPosition,
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition);
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition);
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition);
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @param listener            listener in charge of handling events.
     * @param type                a non-linear 2D position estimator.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<
                    ? extends RadioSource, ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition,
            final FingerprintPositionEstimatorListener<Point2D> listener,
            final NonLinearFingerprintPositionEstimatorType type) {
        switch (type) {
            case THIRD_ORDER:
                return new ThirdOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition,
                        listener);
            case SECOND_ORDER:
                return new SecondOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition,
                        listener);
            case FIRST_ORDER:
            default:
                return new FirstOrderNonLinearFingerprintPositionEstimator2D(
                        locatedFingerprints, fingerprint, sources, initialPosition,
                        listener);
        }
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using default type.
     *
     * @return a non-linear 2D position estimator.
     */
    public static NonLinearFingerprintPositionEstimator2D create() {
        return create(DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using default type.
     *
     * @param listener listener in charge of handling events.
     * @return a non-linear 2D position estimator.
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        return create(listener, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        return create(locatedFingerprints, fingerprint, sources, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param listener            listener in charge of handling events.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        return create(locatedFingerprints, fingerprint, sources, listener,
                DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Point2D initialPosition) {
        return create(locatedFingerprints, fingerprint, sources, initialPosition,
                DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a non-linear 2D position estimator using provided type.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @param listener            listener in charge of handling events.
     * @return a non-linear 2D position estimator.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required).
     */
    public static NonLinearFingerprintPositionEstimator2D create(
            final List<? extends RssiFingerprintLocated<
                    ? extends RadioSource, ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        return create(locatedFingerprints, fingerprint, sources, initialPosition,
                listener, DEFAULT_TYPE);
    }
}
