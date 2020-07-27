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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLateration3DSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLateration3DSolver;

import java.util.List;

/**
 * Linearly estimates 3D position using located radio sources and their RSSI readings at
 * unknown locations.
 * These kind of estimators can be used to determine the 3D position of a given device by
 * getting RSSI readings at an unknown location of different radio sources whose locations
 * are known.
 */
@SuppressWarnings("WeakerAccess")
public class LinearRssiPositionEstimator3D extends LinearRssiPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public LinearRssiPositionEstimator3D() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public LinearRssiPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearRssiPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than
     *                                  the required minimum.
     */
    public LinearRssiPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public LinearRssiPositionEstimator3D(
            final RssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public LinearRssiPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided location radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearRssiPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public LinearRssiPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Sets positions and distances on internal lateration solver.
     *
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @throws IllegalArgumentException if something fails.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected void setPositionsAndDistances(
            final List<Point3D> positions, final List<Double> distances) {
        final int size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        final double[] distancesArray = new double[size];
        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
        }

        try {
            mHomogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray,
                    distancesArray);
            mInhomogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray,
                    distancesArray);
        } catch (final LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }

    /**
     * Initializes lateration solver.
     */
    private void init() {
        mHomogeneousTrilaterationSolver = new HomogeneousLinearLeastSquaresLateration3DSolver(
                mLaterationSolverListener);
        mInhomogeneousTrilaterationSolver = new InhomogeneousLinearLeastSquaresLateration3DSolver(
                mLaterationSolverListener);
    }
}
