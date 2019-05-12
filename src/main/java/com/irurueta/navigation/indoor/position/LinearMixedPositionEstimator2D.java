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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLateration2DSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLateration2DSolver;

import java.util.List;

/**
 * Linearly estimates 2D position using located radio sources and their readings at
 * unknown locations.
 * These kind of estimators can be used to determine the 2D position of a given device by
 * getting ranging readings at an unknown location of different radio sources whose
 * locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class LinearMixedPositionEstimator2D extends
        LinearMixedPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public LinearMixedPositionEstimator2D() {
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
    public LinearMixedPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearMixedPositionEstimator2D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing ranging readings at an unknown location
     *                      for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than
     *                                  the required minimum.
     */
    public LinearMixedPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public LinearMixedPositionEstimator2D(
            MixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources   located radio sources used for lateration.
     * @param listener  listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public LinearMixedPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            MixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint   fingerprint containing ranging readings at an unknown
     *                      location for provided location radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearMixedPositionEstimator2D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            MixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing ranging readings at an unknown
     *                      location for provided located radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public LinearMixedPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            MixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if(mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint2D result = new InhomogeneousPoint2D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Sets positions and distances on internal lateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @throws IllegalArgumentException if something fails.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected void setPositionsAndDistances(List<Point2D> positions,
                                            List<Double> distances) {
        int size = positions.size();
        Point2D[] positionsArray = new InhomogeneousPoint2D[size];
        positionsArray = positions.toArray(positionsArray);

        double[] distancesArray = new double[size];
        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
        }

        try {
            mHomogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray,
                    distancesArray);
            mInhomogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray,
                    distancesArray);
        } catch (LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }

    /**
     * Initializes lateration solver.
     */
    private void init() {
        mHomogeneousTrilaterationSolver = new HomogeneousLinearLeastSquaresLateration2DSolver(
                mLaterationSolverListener);
        mInhomogeneousTrilaterationSolver = new InhomogeneousLinearLeastSquaresLateration2DSolver(
                mLaterationSolverListener);
    }
}
