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

import java.util.List;

/**
 * Base class for robust 3D position estimators using located radio sources and their
 * readings at unknown locations.
 * These kind of estimators can be used to robustly determine the 3D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * locations are known.
 * Implementations of this class should be able to detect and discard outliers in order to
 * find the best solution.
 */
public abstract class RobustMixedPositionEstimator3D extends
        RobustMixedPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public RobustMixedPositionEstimator3D() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustMixedPositionEstimator3D(
            RobustMixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     *
     * @return minimum required number of located radio sources to perform trilateration.
     */
    @Override
    public int getMinRequiredSources() {
        return Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * trilateration solver.
     *
     * @param positions                     positions to be set.
     * @param distances                     distances to be set.
     * @param distanceStandardDeviations    standard deviations of distances to be set.
     * @param distanceQualityScores         distance quality scores or null if not
     *                                      required.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected void setPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            List<Point3D> positions, List<Double> distances,
            List<Double> distanceStandardDeviations,
            List<Double> distanceQualityScores) {
        int size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        double[] distancesArray = new double[size];
        double[] distanceStandardDeviationsArray = new double[size];

        double[] qualityScoresArray = null;
        if (distanceQualityScores != null) {
            qualityScoresArray = new double[size];
        }

        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);

            if (qualityScoresArray != null) {
                qualityScoresArray[i] = distanceQualityScores.get(i);
            }
        }

        try {
            mTrilaterationSolver.setPositionsDistancesAndStandardDeviations(
                    positionsArray, distancesArray, distanceStandardDeviationsArray);

            if (qualityScoresArray != null) {
                mTrilaterationSolver.setQualityScores(qualityScoresArray);
            }
        } catch (LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }
}
