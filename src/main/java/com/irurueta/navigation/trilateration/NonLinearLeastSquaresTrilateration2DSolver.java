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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;

/**
 * Solves a Trilateration problem with an instance of a least squares optimizer.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class NonLinearLeastSquaresTrilateration2DSolver extends NonLinearLeastSquaresTrilaterationSolver<Point2D> {

    /**
     * Constructor.
     */
    public NonLinearLeastSquaresTrilateration2DSolver() {
        super();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start trilateration solving.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances, Point2D initialPosition)
            throws IllegalArgumentException {
        super(positions, distances, initialPosition);
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(TrilaterationSolverListener<Point2D> listener) {
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
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D initialPosition,
            TrilaterationSolverListener<Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions or distances are null, don't have the same length or their
     * length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances, Point2D initialPosition,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(positions, distances, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles) throws IllegalArgumentException {
        super();
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles, Point2D initialPosition)
            throws IllegalArgumentException {
        super(initialPosition);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles, Point2D initialPosition,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(initialPosition, listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations, Point2D initialPosition)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, initialPosition);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations, TrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either positions, distances or standard deviations
     * are null, don't have the same length of their length is smaller than required (2 points).
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations, Point2D initialPosition,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super();
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations, Point2D initialPosition)
            throws IllegalArgumentException {
        super(initialPosition);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(listener);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param initialPosition initial position to start trilateration solving.
     * @param listener listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if circles is null or if length of circles array is less than 2.
     */
    public NonLinearLeastSquaresTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations, Point2D initialPosition,
            TrilaterationSolverListener<Point2D> listener) throws IllegalArgumentException {
        super(initialPosition, listener);
        internalSetCirclesAndStandardDeviations(circles, distanceStandardDeviations);
    }

    /**
     * Gets circles defined by provided positions and distances.
     * @return circles defined by provided positions and distances.
     */
    public Circle[] getCircles() {
        if (mPositions == null) {
            return null;
        }

        Circle[] result = new Circle[mPositions.length];

        for (int i = 0; i < mPositions.length; i++) {
            result[i] = new Circle(mPositions[i], mDistances[i]);
        }
        return result;
    }

    /**
     * Sets circles defining positions and euclidean distances.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     * is less than 2.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setCircles(Circle[] circles) throws IllegalArgumentException, LockedException {
        if(isLocked()) {
            throw new LockedException();
        }
        internalSetCircles(circles);
    }

    /**
     * Sets circles defining positions and euclidean distances along with the standard
     * deviations of provided circles radii.
     * @param circles circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     * 2 or don't have the same length.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setCirclesAndStandardDeviations(Circle[] circles, double[] radiusStandardDeviations)
            throws IllegalArgumentException, LockedException {
        if(isLocked()) {
            throw new LockedException();
        }
        internalSetCirclesAndStandardDeviations(circles, radiusStandardDeviations);
    }

    /**
     * Gets number of dimensions of provided points.
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        getEstimatedPosition(position);
        return position;
    }

    /**
     * Internally sets circles defining positions and euclidean distances.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     * is less than 2.
     */
    private void internalSetCircles(Circle[] circles) throws IllegalArgumentException {
        if (circles == null || circles.length < MIN_POINTS) {
            throw new IllegalArgumentException();
        }

        Point2D[] positions = new Point2D[circles.length];
        double[] distances = new double[circles.length];
        for (int i = 0; i < circles.length; i++) {
            Circle circle = circles[i];
            positions[i] = circle.getCenter();
            distances[i] = circle.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Internally sets circles defining positions and euclidean distances along with the standard
     * deviations of provided circles radii.
     * @param circles circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     * 2 or don't have the same length.
     */
    private void internalSetCirclesAndStandardDeviations(Circle[] circles, double[] radiusStandardDeviations)
            throws IllegalArgumentException {
        if (circles == null || circles.length < MIN_POINTS) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != circles.length) {
            throw new IllegalArgumentException();
        }

        Point2D[] positions = new Point2D[circles.length];
        double[] distances = new double[circles.length];
        for (int i = 0; i < circles.length; i++) {
            Circle circle = circles[i];
            positions[i] = circle.getCenter();
            distances[i] = circle.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                radiusStandardDeviations);
    }
}
