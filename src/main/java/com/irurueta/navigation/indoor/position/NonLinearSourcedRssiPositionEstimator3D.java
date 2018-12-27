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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * 3D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This is a base implementation for all implementations using different orders of
 * Taylor approximation to estimate position.
 */
@SuppressWarnings("WeakerAccess")
public abstract class NonLinearSourcedRssiPositionEstimator3D extends
        NonLinearSourcedRssiPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public NonLinearSourcedRssiPositionEstimator3D() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public NonLinearSourcedRssiPositionEstimator3D(
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
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
     * readings within provided fingerprints (for 3D position estimation 3 located
     * total readings are required among all fingerprints).
     */
    public NonLinearSourcedRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources) {
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
     * readings within provided fingerprints (for 3D position estimation 3 located
     * total readings are required among all fingerprints).
     */
    public NonLinearSourcedRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources,
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param initialPosition initial position to start the solving algorithm or null.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 3D position estimation 3 located
     * total readings are required among all fingerprints).
     */
    public NonLinearSourcedRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources, Point3D initialPosition) {
        super(locatedFingerprints, fingerprint, sources, initialPosition);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param initialPosition initial position to start the solving algorithm or null.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public NonLinearSourcedRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<
            ? extends RadioSource, ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources, Point3D initialPosition,
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, sources, initialPosition, listener);
    }

    /**
     * Gets number of dimensions of points.
     * @return number of dimensions of points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated position or null if not available yet.
     * @return estimated position or null.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        Point3D result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }
}
