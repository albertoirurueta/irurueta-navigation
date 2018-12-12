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

import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly solve the trilateration problem by
 * finding the best pairs of 3D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustTrilateration3DSolver extends RobustTrilaterationSolver<Point3D> {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Linear trilateration solver internally used by a robust algorithm.
     */
    protected LinearLeastSquaresTrilateration3DSolver mLinearSolver;

    /**
     * Non linear trilateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresTrilateration3DSolver mNonLinearSolver;

    /**
     * Positions for linear inner solver used during robust estimation.
     */
    protected Point3D[] mInnerPositions;

    /**
     * Distances for linear inner solver used during robust estimation.
     */
    protected double[] mInnerDistances;

    /**
     * Constructor.
     */
    public RobustTrilateration3DSolver() {
        init();
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustTrilateration3DSolver(
            RobustTrilaterationSolverListener<Point3D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances) {
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
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener) {
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
     * than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public RobustTrilateration3DSolver(Sphere[] spheres) {
        this();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     * than required (4 points) or don't have the same length.
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations) {
        this();
        internalSetSpheresAndStandardDeviations(spheres,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            RobustTrilaterationSolverListener<Point3D> listener) {
        this(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     * than required (4 points) or don't have the same length.
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        this(listener);
        internalSetSpheresAndStandardDeviations(spheres,
                distanceStandardDeviations);
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
     * @throws IllegalArgumentException if sphere is null or length of array of spheres
     * is less than 4.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setSpheres(Sphere[] spheres) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheres(spheres);
    }

    /**
     * Sets spheres defining positions and euclidean distances along with the standard
     * deviations of provided spheres radii.
     * @param spheres spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     * 4 or don' thave the same length.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setSpheresAndStandardDeviations(Sphere[] spheres, double[] radiusStandardDeviations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheresAndStandardDeviations(spheres, radiusStandardDeviations);
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     */
    public static RobustTrilateration3DSolver create(RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver();
            case LMedS:
                return new LMedSRobustTrilateration3DSolver();
            case MSAC:
                return new MSACRobustTrilateration3DSolver();
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver();
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver();
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     */
    public static RobustTrilateration3DSolver create(
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(positions, distances);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (4 points).
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(positions, distances,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(positions, distances,
                        listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length
     * is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param spheres spheres defining positions and distances.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(spheres);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(spheres);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            double[] distanceStandardDeviations, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null or if length of spheres
     * array is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres, listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres, listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres, listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(spheres, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(spheres, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (4 samples).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver();
            case LMedS:
                return new LMedSRobustTrilateration3DSolver();
            case MSAC:
                return new MSACRobustTrilateration3DSolver();
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (4 samples).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     * scores are null, don't have the same length or their length is smaller than
     * required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        positions, distances);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        positions, distances);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
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
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     * deviations are null, don't have the same length or their length is smaller
     * than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        positions, distances, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(positions, distances,
                        listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(positions, distances,
                        listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(positions, distances,
                        listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        positions, distances, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        positions, distances, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres or quality scores are
     * null don't have the same length or their length is less than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        spheres);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        spheres);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres, double[] distanceStandardDeviations,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        spheres, distanceStandardDeviations);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        spheres, distanceStandardDeviations);
        }
    }

    /**
     * Creates a robust 3D trilateration solver.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method robust estimator method.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case LMedS:
                return new LMedSRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case MSAC:
                return new MSACRobustTrilateration3DSolver(spheres,
                        distanceStandardDeviations, listener);
            case PROSAC:
                return new PROSACRobustTrilateration3DSolver(qualityScores,
                        spheres, distanceStandardDeviations, listener);
            case PROMedS:
            default:
                return new PROMedSRobustTrilateration3DSolver(qualityScores,
                        spheres, distanceStandardDeviations, listener);
        }
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @return a new robust 3D trilateration solver.
     */
    public static RobustTrilateration3DSolver create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     */
    public static RobustTrilateration3DSolver create(
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances) {
        return create(positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (4 points).
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, double[] distanceStandardDeviations) {
        return create(positions, distances, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, RobustTrilaterationSolverListener<Point3D> listener) {
        return create(positions, distances, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length
     * is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Point3D[] positions,
            double[] distances, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(positions, distances, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param spheres spheres defining positions and distances.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres) {
        return create(spheres, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            double[] distanceStandardDeviations) {
        return create(spheres, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null or if length of spheres
     * array is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(spheres, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     * is less than required (4 points) or don't have the same length.
     */
    public static RobustTrilateration3DSolver create(Sphere[] spheres,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(spheres, distanceStandardDeviations, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityscores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (4 samples).
     */
    public static RobustTrilateration3DSolver create(double[] qualityscores) {
        return create(qualityscores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, enda or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     * quality scores is less than required minimum (4 samples).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     * scores are null, don't have the same length or their length is smaller
     * than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances) {
        return create(qualityScores, positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations) {
        return create(qualityScores, positions, distances,
                distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
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
     * than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(qualityScores, positions, distances, distanceStandardDeviations,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     * scores or standard deviations are null, don't have the same length or their
     * length is smaller than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(qualityScores, positions, distances, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres or quality scores are
     * null, don't have the same length or their length is less than required
     * (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres) {
        return create(qualityScores, spheres, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres, double[] distanceStandardDeviations) {
        return create(qualityScores, spheres, distanceStandardDeviations,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D trilateration solver using default robust method.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D trilateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     * standard deviations are null, don't have the same length or their length
     * is less than required (4 points).
     */
    public static RobustTrilateration3DSolver create(double[] qualityScores,
            Sphere[] spheres, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener) {
        return create(qualityScores, spheres, distanceStandardDeviations,
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
    protected Point3D attemptRefine(Point3D position) {
        if (mRefineResult && mInliersData != null) {
            BitSet inliers = mInliersData.getInliers();
            int nSamples = mDistances.length;
            int nInliers = mInliersData.getNumInliers();
            Point3D[] inlierPositions = new Point3D[nInliers];
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
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions list where estimated preliminar solution will be stored.
     */
    protected void solvePreliminarSolutions(int[] samplesIndices, List<Point3D> solutions) {
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
     * Internally sets spheres defining positions and euclidean distances.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     * is less than {@link #getMinRequiredPositionsAndDistances()}
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

    /**
     * Internally sets spheres defining positions and euclidean distances along
     * with the standard deviations of provided spheres radii.
     * @param spheres spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     * 4 or don't have the same length.
     */
    private void internalSetSpheresAndStandardDeviations(Sphere[] spheres,
            double[] radiusStandardDeviations) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != spheres.length) {
            throw new IllegalArgumentException();
        }

        Point3D[] positions = new Point3D[spheres.length];
        double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                radiusStandardDeviations);
    }

    /**
     * Setup inner positions and distances.
     */
    private void init() {
        int points = getMinRequiredPositionsAndDistances();
        mInnerPositions = new Point3D[points];
        mInnerDistances = new double[points];

        mLinearSolver = new LinearLeastSquaresTrilateration3DSolver();
        mNonLinearSolver = new NonLinearLeastSquaresTrilateration3DSolver();
    }
}
