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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;

/**
 * Linearly solves the lateration problem using an homogeneous LMSE solution.
 */
public class HomogeneousLinearLeastSquaresLateration2DSolver extends
        HomogeneousLinearLeastSquaresLaterationSolver<Point2D> {

    /**
     * Constructor.
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 points).
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances) {
        super(positions, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver(
            final LaterationSolverListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     *                                  length is smaller than required (3 points).
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver(
            final Point2D[] positions, final double[] distances,
            final LaterationSolverListener<Point2D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 3.
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver(final Circle[] circles) {
        super();
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles  circles defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 3.
     */
    public HomogeneousLinearLeastSquaresLateration2DSolver(
            final Circle[] circles,
            final LaterationSolverListener<Point2D> listener) {
        super(listener);
        internalSetCircles(circles);
    }

    /**
     * Gets circles defined by provided positions and distances.
     *
     * @return circles defined by provided positions and distances.
     */
    public Circle[] getCircles() {
        if (mPositions == null) {
            return null;
        }

        final Circle[] result = new Circle[mPositions.length];

        for (int i = 0; i < mPositions.length; i++) {
            result[i] = new Circle(mPositions[i], mDistances[i]);
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
     * At least 3 positions and distances will be required to linearly solve a 2D problem.
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
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets circles defining positions and Euclidean distances.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     *                                  is less than 3.
     */
    private void internalSetCircles(final Circle[] circles) {
        if (circles == null || circles.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        final Point2D[] positions = new Point2D[circles.length];
        final double[] distances = new double[circles.length];
        for (int i = 0; i < circles.length; i++) {
            final Circle circle = circles[i];
            positions[i] = circle.getCenter();
            distances[i] = circle.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }
}
