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
package com.irurueta.navigation.lateration;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;

/**
 * Linearly solves the lateration problem using an homogeneous LMSE solution.
 */
@SuppressWarnings("DuplicatedCode")
public class HomogeneousLinearLeastSquaresLateration3DSolver extends
        HomogeneousLinearLeastSquaresLaterationSolver<Point3D> {

    /**
     * Constructor.
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver(final Point3D[] positions, final double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver(final LaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances, final LaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 4.
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver(final Sphere[] spheres) {
        super();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres  spheres defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 4.
     */
    public HomogeneousLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres, final LaterationSolverListener<Point3D> listener) {
        super(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Gets spheres defined by provided positions and distances.
     *
     * @return spheres defined by provided positions and distances.
     */
    public Sphere[] getSpheres() {
        if (positions == null) {
            return null;
        }

        final var result = new Sphere[positions.length];

        for (var i = 0; i < positions.length; i++) {
            result[i] = new Sphere(positions[i], distances[i]);
        }
        return result;
    }

    /**
     * Sets spheres defining positions and Euclidean distances.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     *                                  is less than 2.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setSpheres(final Sphere[] spheres) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheres(spheres);
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Minimum required number of positions and distances.
     * At least 4 positions and distances will be required to linearly solve a 3D problem.
     *
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (estimatedPositionCoordinates == null) {
            return null;
        }

        final var position = new InhomogeneousPoint3D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets spheres defining positions and Euclidean distances.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     *                                  is less than 4.
     */
    private void internalSetSpheres(final Sphere[] spheres) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        final var positions = new Point3D[spheres.length];
        final var distances = new double[spheres.length];
        for (var i = 0; i < spheres.length; i++) {
            final var sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }
}
