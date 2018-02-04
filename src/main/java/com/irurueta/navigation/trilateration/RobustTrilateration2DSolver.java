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
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;

import java.util.BitSet;

/**
 * This is an abstract class to robustly solve the trilateration problem by
 * finding the best pairs of 2D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("WeakerAccess")
public abstract class RobustTrilateration2DSolver extends RobustTrilaterationSolver<Point2D> {

    /**
     * Linear trilateration solver internally used by a robust algorithm.
     */
    protected LinearLeastSquaresTrilateration2DSolver mLinearSolver =
            new LinearLeastSquaresTrilateration2DSolver();

    /**
     * Non linear trilateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresTrilateration2DSolver mNonLinearSolver =
            new NonLinearLeastSquaresTrilateration2DSolver();

    /**
     * Positions for linear inner solver used during robust estimation.
     */
    protected Point2D[] mInnerPositions;

    /**
     * Distances for linear inner solver used during robust estimation.
     */
    protected double[] mInnerDistances;

    /**
     * Constructor.
     */
    public RobustTrilateration2DSolver() {
        init();
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustTrilateration2DSolver(
            RobustTrilaterationSolverListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public RobustTrilateration2DSolver(Point2D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public RobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public RobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public RobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     * is less than required (3 points).
     */
    public RobustTrilateration2DSolver(Circle[] circles) throws IllegalArgumentException {
        this();
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     * than required (3 points) or don't have the same length.
     */
    public RobustTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        this();
        internalSetCirclesAndStandardDeviations(circles,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     * is less than required (3 points).
     */
    public RobustTrilateration2DSolver(Circle[] circles,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     * than required (3 points) or don't have the same length.
     */
    public RobustTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetCirclesAndStandardDeviations(circles,
                distanceStandardDeviations);
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
     * Minimum required number of positions and distances.
     * At least 3 positions will be required.
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
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
    public void setCircles(Circle[] circles) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
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
     * Attempts to refine estimated position if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     * @param position position estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or provided non-refined
     * estimated position if not requested or refinement failed.
     */
    protected Point2D attemptRefine(Point2D position) {
        if (mRefineResult && mInliersData != null) {
            BitSet inliers = mInliersData.getInliers();
            int nSamples = inliers.length();
            Point2D[] inlierPositions = new Point2D[nSamples];
            double[] inlierDistances = new double[nSamples];
            double[] inlierStandardDeviations = null;
            if (mDistanceStandardDeviations != null) {
                inlierStandardDeviations = new double[nSamples];
            }
            int pos = 0;
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    //sample is inlier
                    inlierPositions[pos] = mPositions[i];
                    inlierDistances[pos] = mDistances[i];
                    if (inlierStandardDeviations != null) {
                        inlierStandardDeviations[pos] = mDistanceStandardDeviations[i];
                    }
                    pos++;
                }
            }

            try {
                mNonLinearSolver.setInitialPosition(position);
                if (inlierStandardDeviations != null) {
                    mNonLinearSolver.setPositionsDistancesAndStandardDeviations(
                            inlierPositions, inlierDistances, inlierStandardDeviations);
                } else {
                    mNonLinearSolver.setPositionsAndDistances(
                            inlierPositions, inlierDistances);
                }
                mNonLinearSolver.solve();

                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = mNonLinearSolver.getCovariance();
                }

                return mEstimatedPosition = mNonLinearSolver.getEstimatedPosition();
            } catch (Exception e) {
                //refinement failed, so we return input value
                return mEstimatedPosition = position;
            }
        } else {
            return mEstimatedPosition = position;
        }
    }

    /**
     * Internally sets circles defining positions and euclidean distances.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     * is less than {@link #getMinRequiredPositionsAndDistances}.
     */
    private void internalSetCircles(Circle[] circles) throws IllegalArgumentException {
        if (circles == null || circles.length < getMinRequiredPositionsAndDistances()) {
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
    private void internalSetCirclesAndStandardDeviations(Circle[] circles,
            double[] radiusStandardDeviations) throws IllegalArgumentException {
        if (circles == null || circles.length < getMinRequiredPositionsAndDistances()) {
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

    /**
     * Setup inner positions and distances.
     */
    private void init() {
        int points = getMinRequiredPositionsAndDistances();
        mInnerPositions = new Point2D[points];
        mInnerDistances = new double[points];

        mLinearSolver = new LinearLeastSquaresTrilateration2DSolver();
        mNonLinearSolver = new NonLinearLeastSquaresTrilateration2DSolver();
    }
}
