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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;

/**
 * Solves a Trilateration problem with an instance of the least squares optimizer.
 */
public class NonLinearLeastSquaresLateration2DSolver extends NonLinearLeastSquaresLaterationSolver<Point2D> {

    /**
     * Constructor.
     */
    public NonLinearLeastSquaresLateration2DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(final Point2D[] positions, final double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     */
    public NonLinearLeastSquaresLateration2DSolver(final Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     *
     * @param positions       known positions of static nodes.
     * @param distances       euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start lateration solving.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final Point2D initialPosition) {
        super(positions, distances, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLateration2DSolver(final LaterationSolverListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final LaterationSolverListener<Point2D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D initialPosition, final LaterationSolverListener<Point2D> listener) {
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
     *                                  length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final Point2D initialPosition,
            final LaterationSolverListener<Point2D> listener) {
        super(positions, distances, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(final Circle[] circles) {
        super();
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles         circles defining positions and distances.
     * @param initialPosition initial position to start lateration solving.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(final Circle[] circles, final Point2D initialPosition) {
        super(initialPosition);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles  circles defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final LaterationSolverListener<Point2D> listener) {
        super(listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles         circles defining positions and distances.
     * @param initialPosition initial position to start lateration solving.
     * @param listener        listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final Point2D initialPosition, final LaterationSolverListener<Point2D> listener) {
        super(initialPosition, listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     *                                  are null, don't have the same length of their length is smaller than required
     *                                  (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final double[] distanceStandardDeviations) {
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
     *                                  are null, don't have the same length of their length is smaller than required
     *                                  (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final Point2D initialPosition) {
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
     *                                  are null, don't have the same length of their length is smaller than required
     *                                  (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final LaterationSolverListener<Point2D> listener) {
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
     *                                  are null, don't have the same length of their length is smaller than required
     *                                  (2 points).
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final Point2D initialPosition, final LaterationSolverListener<Point2D> listener) {
        super(positions, distances, distanceStandardDeviations, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations) {
        super();
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations, final Point2D initialPosition) {
        super(initialPosition);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations,
            final LaterationSolverListener<Point2D> listener) {
        super(listener);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition            initial position to start lateration solving.
     * @param listener                   listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations, final Point2D initialPosition,
            final LaterationSolverListener<Point2D> listener) {
        super(initialPosition, listener);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Gets circles defined by provided positions and distances.
     *
     * @return circles defined by provided positions and distances.
     */
    public Circle[] getCircles() {
        if (positions == null) {
            return null;
        }

        final var result = new Circle[positions.length];

        for (var i = 0; i < positions.length; i++) {
            result[i] = new Circle(positions[i], distances[i]);
        }
        return result;
    }

    /**
     * Sets circles defining positions and euclidean distances.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     *                                  is less than 2.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setCircles(final Circle[] circles) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCircles(circles);
    }

    /**
     * Sets circles defining positions and Euclidean distances along with the standard
     * deviations of provided circles radii.
     *
     * @param circles                  circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     *                                  2 or don't have the same length.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setCirclesAndStandardDeviations(final Circle[] circles, final double[] radiusStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCirclesAndStandardDeviations(circles, radiusStandardDeviations);
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Minimum required number of positions and distances.
     * At least 3 positions and distances will be required to solve a 2D problem.
     *
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (estimatedPositionCoordinates == null) {
            return null;
        }

        final var position = new InhomogeneousPoint2D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets circles defining positions and Euclidean distances.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     *                                  is less than 2.
     */
    private void internalSetCircles(final Circle[] circles) {
        if (circles == null || circles.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        final var positions = new Point2D[circles.length];
        final var distances = new double[circles.length];
        for (var i = 0; i < circles.length; i++) {
            final var circle = circles[i];
            positions[i] = circle.getCenter();
            distances[i] = circle.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Internally sets circles defining positions and Euclidean distances along with the standard
     * deviations of provided circles radii.
     *
     * @param circles                  circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     *                                  2 or don't have the same length.
     */
    private void internalSetCirclesAndStandardDeviations(
            final Circle[] circles, final double[] radiusStandardDeviations) {
        if (circles == null || circles.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != circles.length) {
            throw new IllegalArgumentException();
        }

        final var positions = new Point2D[circles.length];
        final var distances = new double[circles.length];
        for (var i = 0; i < circles.length; i++) {
            final var circle = circles[i];
            positions[i] = circle.getCenter();
            distances[i] = circle.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances, radiusStandardDeviations);
    }
}
