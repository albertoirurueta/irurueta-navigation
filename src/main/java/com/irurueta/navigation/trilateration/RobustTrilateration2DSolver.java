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
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly solve the trilateration problem by
 * finding the best pairs of 2D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustTrilateration2DSolver extends RobustTrilaterationSolver<Point2D> {

    /**
     * Linear trilateration solver internally used by a robust algorithm.
     */
    protected LinearLeastSquaresTrilateration2DSolver mLinearSolver;

    /**
     * Non linear trilateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresTrilateration2DSolver mNonLinearSolver;

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
    public RobustTrilateration2DSolver(Point2D[] positions, double[] distances) {
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
            double[] distanceStandardDeviations) {
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
            RobustTrilaterationSolverListener<Point2D> listener) {
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
            RobustTrilaterationSolverListener<Point2D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     * is less than required (3 points).
     */
    public RobustTrilateration2DSolver(Circle[] circles) {
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
            double[] distanceStandardDeviations) {
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
            RobustTrilaterationSolverListener<Point2D> listener) {
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
            RobustTrilaterationSolverListener<Point2D> listener) {
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
     * is less than 3.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setCircles(Circle[] circles) throws LockedException {
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
     * 3 or don't have the same length.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setCirclesAndStandardDeviations(Circle[] circles, double[] radiusStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCirclesAndStandardDeviations(circles, radiusStandardDeviations);
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     */
    public static RobustTrilateration2DSolver create(RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver();
            case LMedS:
                return new LMedSRobustTrilateration2DSolver();
            case MSAC:
                return new MSACRobustTrilateration2DSolver();
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver();
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver();
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     */
    public static RobustTrilateration2DSolver create(
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(positions, distances);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (3 points).
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaler than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(positions, distances,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(positions, distances,
                        listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length
     * is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param circles circles defining positions and distances.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(circles);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(circles);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            double[] distanceStandardDeviations, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null or if length of circles
     * array is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles, listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles, listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles, listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(circles, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(circles, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (3 samples).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver();
            case LMedS:
                return new LMedSRobustTrilateration2DSolver();
            case MSAC:
                return new MSACRobustTrilateration2DSolver();
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (3 samples).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     * scores are null, don't have the same length or their length is smaller than
     * required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        positions, distances);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or the
     * length is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     * deviations are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        positions, distances, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        positions, distances, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles or quality scores are
     * null don't have the same length or their length is less than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        circles);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        circles);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 2D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration2DSolver(circles,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration2DSolver(qualityScores,
                        circles, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @return a new robust 2D trilateration solver.
     */
    public static RobustTrilateration2DSolver create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     */
    public static RobustTrilateration2DSolver create(
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances) {
        return create(positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (3 points).
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, double[] distanceStandardDeviations) {
        return create(positions, distances, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, RobustTrilaterationSolverListener<Point2D> listener) {
        return create(positions, distances, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length
     * is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(Point2D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(positions, distances, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param circles circles defining positions and distances.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles) {
        return create(circles, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            double[] distanceStandardDeviations) {
        return create(circles, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null or if length of circles
     * array is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(circles, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if circles is null, length of circles array
     * is less than required (3 points) or don't have the same length.
     */
    public static RobustTrilateration2DSolver create(Circle[] circles,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(circles, distanceStandardDeviations, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required (3 samples).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (3 samples).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     * scores are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances) {
        return create(qualityScores, positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distance from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations) {
        return create(qualityScores, positions, distances,
                distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     * deviations are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create (qualityScores, positions, distances,
                distanceStandardDeviations,listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static ndoes to mobile node.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Point2D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(qualityScores, positions, distances, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles or quality scores are
     * null, don't have the same length or their length is less than required
     * (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles) {
        return create(qualityScores, circles, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations) {
        return create(qualityScores, circles, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 2D trilateration solver.
     * @throws IllegalArgumentException if either circles, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (3 points).
     */
    public static RobustTrilateration2DSolver create(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener) {
        return create(qualityScores, circles, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
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
            int nSamples = mDistances.length;
            int nInliers = mInliersData.getNumInliers();
            Point2D[] inlierPositions = new Point2D[nInliers];
            double[] inlierDistances = new double[nInliers];
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

                return mEstimatedPosition = mNonLinearSolver.getEstimatedPosition();
            } catch (Exception e) {
                //refinement failed, so we return input value
                mCovariance = null;
                return mEstimatedPosition = position;
            }
        } else {
            mCovariance = null;
            return mEstimatedPosition = position;
        }
    }

    /**
     * Solves a preliminar solution for a subset of samples picked by a robust estimator.
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions list where estimated preliminar solution will be stored.
     */
    protected void solvePreliminarSolutions(int[] samplesIndices, List<Point2D> solutions) {
        try {
            int length = samplesIndices.length;
            int index;
            for (int i = 0; i < length; i++) {
                index = samplesIndices[i];
                mInnerPositions[i] = mPositions[index];
                mInnerDistances[i] = mDistances[index];
            }

            mLinearSolver.setPositionsAndDistances(mInnerPositions, mInnerDistances);
            mLinearSolver.solve();
            solutions.add(mLinearSolver.getEstimatedPosition());
        } catch (NavigationException ignore) {
            //if anything fails, no solution is added
        }
    }

    /**
     * Internally sets circles defining positions and euclidean distances.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or length of array of circles
     * is less than {@link #getMinRequiredPositionsAndDistances}.
     */
    private void internalSetCircles(Circle[] circles) {
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
     * 3 or don't have the same length.
     */
    private void internalSetCirclesAndStandardDeviations(Circle[] circles,
            double[] radiusStandardDeviations) {
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
