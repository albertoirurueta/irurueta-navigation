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
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;

/**
 * Linearly solves the trilateration problem.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class LinearLeastSquaresTrilateration3DSolver extends LinearLeastSquaresTrilaterationSolver<Point3D> {

    /**
     * Constructor.
     */
    public LinearLeastSquaresTrilateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public LinearLeastSquaresTrilateration3DSolver(Point3D[] positions, double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    public LinearLeastSquaresTrilateration3DSolver(TrilaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public LinearLeastSquaresTrilateration3DSolver(Point3D[] positions, double[] distances,
            TrilaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public LinearLeastSquaresTrilateration3DSolver(Sphere[] spheres) {
        super();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public LinearLeastSquaresTrilateration3DSolver(Sphere[] spheres,
            TrilaterationSolverListener<Point3D> listener) {
        super(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Gets spheres defined by provided positions and distances.
     * @return spheres defined by provided positions and distances.
     */
    public Sphere[] getSpheres() {
        if (mPositions == null) {
            return null;
        }

        Sphere[] result = new Sphere[mPositions.length];

        for (int i = 0; i < mPositions.length; i++) {
            result[i] = new Sphere(mPositions[i], mDistances[i]);
        }
        return result;
    }

    /**
     * Sets spheres defining positions and euclidean distances.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     * is less than 2.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setSpheres(Sphere[] spheres) throws LockedException {
        if(isLocked()) {
            throw new LockedException();
        }
        internalSetSpheres(spheres);
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
     * At least 4 positions and distances will be required to linearly solve a 3D problem.
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets spheres defining positions and euclidean distances.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     * is less than 2.
     */
    private void internalSetSpheres(Sphere[] spheres) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        Point3D[] positions = new Point3D[spheres.length];
        double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }
}
