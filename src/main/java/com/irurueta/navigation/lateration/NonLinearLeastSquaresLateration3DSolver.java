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
package com.irurueta.navigation.lateration;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;

/**
 * Solves a Trilateration problem with an instance of a least squares optimizer.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class NonLinearLeastSquaresLateration3DSolver extends NonLinearLeastSquaresLaterationSolver<Point3D> {

    /**
     * Constructor.
     */
    public NonLinearLeastSquaresLateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     */
    public NonLinearLeastSquaresLateration3DSolver(final Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     *
     * @param positions       known positions of static nodes.
     * @param distances       euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start lateration solving.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions,
            final double[] distances, final Point3D initialPosition) {
        super(positions, distances, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final LaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final LaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D initialPosition,
            final LaterationSolverListener<Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param positions       known positions of static nodes.
     * @param distances       euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 for 2D points or 4 for 3D points) or fitter is null.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions,
            final double[] distances, final Point3D initialPosition,
            final LaterationSolverListener<Point3D> listener) {
        super(positions, distances, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(final Sphere[] spheres) {
        super();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres         spheres defining positions and distances.
     * @param initialPosition initial position to start lateration solving.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres, final Point3D initialPosition) {
        super(initialPosition);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres  spheres defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres,
            final LaterationSolverListener<Point3D> listener) {
        super(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres         spheres defining positions and distances.
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres, final Point3D initialPosition,
            final LaterationSolverListener<Point3D> listener) {
        super(initialPosition, listener);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final Point3D initialPosition) {
        super(positions, distances, distanceStandardDeviations, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final LaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final Point3D initialPosition,
            final LaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres,
            final double[] distanceStandardDeviations) {
        super();
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres,
            final double[] distanceStandardDeviations, final Point3D initialPosition) {
        super(initialPosition);
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres,
            final double[] distanceStandardDeviations,
            final LaterationSolverListener<Point3D> listener) {
        super(listener);
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array is less than 2.
     */
    public NonLinearLeastSquaresLateration3DSolver(
            final Sphere[] spheres,
            final double[] distanceStandardDeviations, final Point3D initialPosition,
            final LaterationSolverListener<Point3D> listener) {
        super(initialPosition, listener);
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
    }

    /**
     * Gets spheres defined by provided positions and distances.
     *
     * @return spheres defined by provided positions and distances.
     */
    public Sphere[] getSpheres() {
        if (mPositions == null) {
            return null;
        }

        final Sphere[] result = new Sphere[mPositions.length];

        for (int i = 0; i < mPositions.length; i++) {
            result[i] = new Sphere(mPositions[i], mDistances[i]);
        }
        return result;
    }

    /**
     * Sets spheres defining positions and euclidean distances.
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
     * Sets spheres defining positions and euclidean distances along with the standard
     * deviations of provided spheres radii.
     *
     * @param spheres                  spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     *                                  2 or don't have the same length.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setSpheresAndStandardDeviations(
            final Sphere[] spheres, final double[] radiusStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheresAndStandardDeviations(spheres, radiusStandardDeviations);
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return always returns 3 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Minimum required number of positions and distances.
     * At least 4 positions and distances will be required to solve a 3D problem.
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
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets spheres defining positions and euclidean distances.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     *                                  is less than 2.
     */
    public void internalSetSpheres(final Sphere[] spheres) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        final Point3D[] positions = new Point3D[spheres.length];
        final double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            final Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Internally sets spheres defining positions and euclidean distances along with the standard
     * deviations of provided spheres radii.
     *
     * @param spheres                  spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     *                                  2 or don't have the same length.
     */
    private void internalSetSpheresAndStandardDeviations(
            final Sphere[] spheres,
            final double[] radiusStandardDeviations) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != spheres.length) {
            throw new IllegalArgumentException();
        }

        final Point3D[] positions = new Point3D[spheres.length];
        final double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            final Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                radiusStandardDeviations);

    }
}
