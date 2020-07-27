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
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly solve the lateration problem by
 * finding the best pairs of 2D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustLateration2DSolver extends RobustLaterationSolver<Point2D> {

    /**
     * Linear lateration solver internally used by a robust algorithm.
     */
    protected InhomogeneousLinearLeastSquaresLateration2DSolver mInhomogeneousLinearSolver;

    /**
     * Homogeneous linear lateration solver internally used by a robust algorithm.
     */
    protected HomogeneousLinearLeastSquaresLateration2DSolver mHomogeneousLinearSolver;

    /**
     * Non linear lateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresLateration2DSolver mNonLinearSolver;

    /**
     * Positions for linear inner solver used during robust estimation.
     */
    protected Point2D[] mInnerPositions;

    /**
     * Distances for linear inner solver used during robust estimation.
     */
    protected double[] mInnerDistances;

    /**
     * Standard deviations for non-linear inner solver used during robut estimation.
     */
    protected double[] mInnerDistanceStandardDeviations;

    /**
     * Constructor.
     */
    public RobustLateration2DSolver() {
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustLateration2DSolver(
            final RobustLaterationSolverListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (3 points).
     */
    public RobustLateration2DSolver(final Point2D[] positions, final double[] distances) {
        super(positions, distances);
        init();
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (3 points).
     */
    public RobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                    final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
        init();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events such as when estimation starts,
     *                  ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (3 points).
     */
    public RobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                    final RobustLaterationSolverListener<Point2D> listener) {
        super(positions, distances, listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation starts,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length is smaller
     *                                  than required (3 points).
     */
    public RobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                    final double[] distanceStandardDeviations,
                                    final RobustLaterationSolverListener<Point2D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     *                                  is less than required (3 points).
     */
    public RobustLateration2DSolver(final Circle[] circles) {
        this();
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     *                                  than required (3 points) or don't have the same length.
     */
    public RobustLateration2DSolver(final Circle[] circles,
                                    final double[] distanceStandardDeviations) {
        this();
        internalSetCirclesAndStandardDeviations(circles,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param circles  circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     *                                  is less than required (3 points).
     */
    public RobustLateration2DSolver(final Circle[] circles,
                                    final RobustLaterationSolverListener<Point2D> listener) {
        this(listener);
        internalSetCircles(circles);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation starts,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     *                                  than required (3 points) or don't have the same length.
     */
    public RobustLateration2DSolver(final Circle[] circles,
                                    final double[] distanceStandardDeviations,
                                    RobustLaterationSolverListener<Point2D> listener) {
        this(listener);
        internalSetCirclesAndStandardDeviations(circles,
                distanceStandardDeviations);
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
     * At least 3 positions will be required.
     *
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinRequiredPositionsAndDistances()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if instance is busy solving the lateration problem.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinRequiredPositionsAndDistances()}.
     */
    @Override
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        super.setPreliminarySubsetSize(preliminarySubsetSize);

        mInnerPositions = new Point2D[preliminarySubsetSize];
        mInnerDistances = new double[preliminarySubsetSize];
        mInnerDistanceStandardDeviations = new double[preliminarySubsetSize];
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
     *                                  is less than 3.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setCircles(final Circle[] circles) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCircles(circles);
    }

    /**
     * Sets circles defining positions and euclidean distances along with the standard
     * deviations of provided circles radii.
     *
     * @param circles                  circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     *                                  3 or don't have the same length.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setCirclesAndStandardDeviations(
            final Circle[] circles, final double[] radiusStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCirclesAndStandardDeviations(circles, radiusStandardDeviations);
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param method robust estimator method.
     * @return a new robust 2D lateration solver.
     */
    public static RobustLateration2DSolver create(final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver();
            case LMedS:
                return new LMedSRobustLateration2DSolver();
            case MSAC:
                return new MSACRobustLateration2DSolver();
            case PROSAC:
                return new PROSACRobustLateration2DSolver();
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver();
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a new robust 2D lateration solver.
     */
    public static RobustLateration2DSolver create(
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method    robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions,
            final double[] distances, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(positions, distances);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (3 points).
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaler than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param method    robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(positions, distances,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(positions, distances,
                        listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param circles circles defining positions and distances.
     * @param method  robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(circles);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(circles);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final double[] distanceStandardDeviations,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param circles  circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null or if length of circles
     *                                  array is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles, listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles, listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles, listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(circles, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(circles, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param method        robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (3 samples).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver();
            case LMedS:
                return new LMedSRobustLateration2DSolver();
            case MSAC:
                return new MSACRobustLateration2DSolver();
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (3 samples).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node to be
     *                      estimated.
     * @param method        robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     *                                  scores are null, don't have the same length or their length is smaller than
     *                                  required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions,
            final double[] distances, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        positions, distances);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or the
     *                                  length is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     *                                  deviations are null, don't have the same length or their length is smaller
     *                                  than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        positions, distances, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        positions, distances, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles       circles defining positions and distances.
     * @param method        robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles or quality scores are
     *                                  null don't have the same length or their length is less than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Circle[] circles,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        circles);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        circles);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Circle[] circles,
            final double[] distanceStandardDeviations, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Circle[] circles,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustLateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustLateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustLateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @return a new robust 2D lateration solver.
     */
    public static RobustLateration2DSolver create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     */
    public static RobustLateration2DSolver create(
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances) {
        return create(positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (3 points).
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        return create(positions, distances, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(positions, distances, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(positions, distances, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param circles circles defining positions and distances.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(final Circle[] circles) {
        return create(circles, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final double[] distanceStandardDeviations) {
        return create(circles, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param circles  circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null or if length of circles
     *                                  array is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final RobustLaterationSolverListener<Point2D> listener) {
        return create(circles, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     *                                  is less than required (3 points) or don't have the same length.
     */
    public static RobustLateration2DSolver create(
            final Circle[] circles, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(circles, distanceStandardDeviations, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required (3 samples).
     */
    public static RobustLateration2DSolver create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (3 samples).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node to be
     *                      estimated.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     *                                  scores are null, don't have the same length or their length is smaller
     *                                  than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions,
            final double[] distances) {
        return create(qualityScores, positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distance from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions,
            final double[] distances, final double[] distanceStandardDeviations) {
        return create(qualityScores, positions, distances,
                distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     *                                  deviations are null, don't have the same length or their length is smaller
     *                                  than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(qualityScores, positions, distances,
                distanceStandardDeviations, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static ndoes to mobile node.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Point2D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(qualityScores, positions, distances, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles       circles defining positions and distances.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles or quality scores are
     *                                  null, don't have the same length or their length is less than required
     *                                  (3 points).
     */
    public static RobustLateration2DSolver create(final double[] qualityScores,
                                                  final Circle[] circles) {
        return create(qualityScores, circles, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Circle[] circles,
            final double[] distanceStandardDeviations) {
        return create(qualityScores, circles, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 2D lateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (3 points).
     */
    public static RobustLateration2DSolver create(
            final double[] qualityScores, final Circle[] circles,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener) {
        return create(qualityScores, circles, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Attempts to refine estimated position if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     *
     * @param position position estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or provided non-refined
     * estimated position if not requested or refinement failed.
     */
    protected Point2D attemptRefine(final Point2D position) {
        if (mRefineResult && mInliersData != null) {
            final BitSet inliers = mInliersData.getInliers();
            final int nSamples = mDistances.length;
            final int nInliers = mInliersData.getNumInliers();
            final Point2D[] inlierPositions = new Point2D[nInliers];
            final double[] inlierDistances = new double[nInliers];
            double[] inlierStandardDeviations = null;
            if (mDistanceStandardDeviations != null) {
                inlierStandardDeviations = new double[nInliers];
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
                } else {
                    mCovariance = null;
                }

                mEstimatedPosition = mNonLinearSolver.getEstimatedPosition();
            } catch (Exception e) {
                //refinement failed, so we return input value
                mCovariance = null;
                mEstimatedPosition = position;
            }
        } else {
            mCovariance = null;
            mEstimatedPosition = position;
        }

        return mEstimatedPosition;
    }

    /**
     * Solves a preliminar solution for a subset of samples picked by a robust estimator.
     *
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions      list where estimated preliminar solution will be stored.
     */
    protected void solvePreliminarSolutions(
            final int[] samplesIndices, final List<Point2D> solutions) {
        try {
            final int length = samplesIndices.length;
            int index;
            for (int i = 0; i < length; i++) {
                index = samplesIndices[i];
                mInnerPositions[i] = mPositions[index];
                mInnerDistances[i] = mDistances[index];
                mInnerDistanceStandardDeviations[i] = mDistanceStandardDeviations != null ?
                        mDistanceStandardDeviations[index] :
                        NonLinearLeastSquaresLaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;
            }

            Point2D estimatedPosition = mInitialPosition;
            if (mUseLinearSolver) {
                if (mUseHomogeneousLinearSolver) {
                    mHomogeneousLinearSolver.setPositionsAndDistances(mInnerPositions, mInnerDistances);
                    mHomogeneousLinearSolver.solve();
                    estimatedPosition = mHomogeneousLinearSolver.getEstimatedPosition();
                } else {
                    mInhomogeneousLinearSolver.setPositionsAndDistances(mInnerPositions, mInnerDistances);
                    mInhomogeneousLinearSolver.solve();
                    estimatedPosition = mInhomogeneousLinearSolver.getEstimatedPosition();
                }
            }

            if (mRefinePreliminarySolutions || estimatedPosition == null) {
                mNonLinearSolver.setInitialPosition(estimatedPosition);
                if (mDistanceStandardDeviations != null) {
                    mNonLinearSolver.setPositionsDistancesAndStandardDeviations(mInnerPositions,
                            mInnerDistances, mInnerDistanceStandardDeviations);
                } else {
                    mNonLinearSolver.setPositionsAndDistances(mInnerPositions, mInnerDistances);
                }
                mNonLinearSolver.solve();
                estimatedPosition = mNonLinearSolver.getEstimatedPosition();
            }

            solutions.add(estimatedPosition);
        } catch (final NavigationException ignore) {
            //if anything fails, no solution is added
        }
    }

    /**
     * Internally sets circles defining positions and euclidean distances.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     *                                  is less than {@link #getMinRequiredPositionsAndDistances}.
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

    /**
     * Internally sets circles defining positions and euclidean distances along with the standard
     * deviations of provided circles radii.
     *
     * @param circles                  circles defining positions and distances.
     * @param radiusStandardDeviations standard deviations of circles radii.
     * @throws IllegalArgumentException if circles is null, length of arrays is less than
     *                                  3 or don't have the same length.
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

        final Point2D[] positions = new Point2D[circles.length];
        final double[] distances = new double[circles.length];
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
        final int points = getMinRequiredPositionsAndDistances();
        mPreliminarySubsetSize = points;
        mInnerPositions = new Point2D[points];
        mInnerDistances = new double[points];
        mInnerDistanceStandardDeviations = new double[points];

        mInhomogeneousLinearSolver = new InhomogeneousLinearLeastSquaresLateration2DSolver();
        mHomogeneousLinearSolver = new HomogeneousLinearLeastSquaresLateration2DSolver();
        mNonLinearSolver = new NonLinearLeastSquaresLateration2DSolver();
    }
}
