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
package com.irurueta.navigation.trilateration;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;

/**
 * This is an abstract class to robustly solve the trilateration problem by
 * finding the best pairs of 3D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class RobustTrilateration3DSolver extends RobustTrilaterationSolver<Point3D> {

    /**
     * Linear trilateration solver internally used by a robust algorithm.
     */
    protected LinearLeastSquaresTrilateration3DSolver mLinearSolver =
            new LinearLeastSquaresTrilateration3DSolver();

    /**
     * Non linear trilateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresTrilateration3DSolver mNonLinearSolver =
            new NonLinearLeastSquaresTrilateration3DSolver();

    /**
     * Constructor.
     */
    public RobustTrilateration3DSolver() { }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustTrilateration3DSolver(
            RobustTrilaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
    }

    /**
     * Gets number of dimensions of provided points.
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Minimum required number of positions and distances.
     * At least 3 positions will be required.
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }
}
