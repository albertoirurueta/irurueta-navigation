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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Position and radio source estimator in 3D based only on located fingerprints
 * containing RSSI readings.
 * This class estimates 3D position of a new fingerprint and the 3D position of all
 * radio sources associated to fingerprints whose location is known.
 */
public class NonLinearFingerprintPositionAndRadioSourceEstimator3D extends
        NonLinearFingerprintPositionAndRadioSourceEstimator<Point3D> {

    /**
     * Constructor.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            FingerprintPositionAndRadioSourceEstimatorListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
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
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            FingerprintPositionAndRadioSourceEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            Point3D initialPosition) {
        super(locatedFingerprints, fingerprint, initialPosition);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            Point3D initialPosition,
            FingerprintPositionAndRadioSourceEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint, initialLocatedSources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> initialLocatedSources,
            FingerprintPositionAndRadioSourceEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, initialLocatedSources, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            Point3D initialPosition,
            List<? extends RadioSourceLocated<Point3D>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint, initialPosition, initialLocatedSources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            Point3D initialPosition,
            List<? extends RadioSourceLocated<Point3D>> initialLocatedSources,
            FingerprintPositionAndRadioSourceEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, initialPosition, initialLocatedSources,
                listener);
    }

    /**
     * Propagates provided variances into RSSI differences.
     * @param pathlossExponent path-loss exponent.
     * @param fingerprintPosition position of closest located fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition position to be estimated. Usually this is equal to the
     *                          initial position used by a non linear algorithm.
     * @param pathlossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null if
     *                                      unknown.
     * @return variance of RSSI difference measured at non located fingerprint reading.
     */
    @Override
    protected Double propagateVariances(double pathlossExponent, Point3D fingerprintPosition,
            Point3D radioSourcePosition, Point3D estimatedPosition,
            Double pathlossExponentVariance, Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance) {
        //TODO: finish implementation
        return null;
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
     * Create a point.
     * @return point to be created.
     */
    @Override
    protected Point3D createPoint() {
        return new InhomogeneousPoint3D();
    }
}
