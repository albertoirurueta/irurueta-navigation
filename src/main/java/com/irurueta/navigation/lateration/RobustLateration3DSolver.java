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

import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class to robustly solve the lateration problem by
 * finding the best pairs of 3D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class RobustLateration3DSolver extends RobustLaterationSolver<Point3D> {

    /**
     * Inhomogeneous linear lateration solver internally used by a robust algorithm.
     */
    protected InhomogeneousLinearLeastSquaresLateration3DSolver inhomogeneousLinearSolver;

    /**
     * Homogeneous linear lateration solver internally used by a robust algorithm.
     */
    protected HomogeneousLinearLeastSquaresLateration3DSolver homogeneousLinearSolver;

    /**
     * Non-linear lateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresLateration3DSolver nonLinearSolver;

    /**
     * Positions for linear inner solver used during robust estimation.
     */
    protected Point3D[] innerPositions;

    /**
     * Distances for linear inner solver used during robust estimation.
     */
    protected double[] innerDistances;

    /**
     * Standard deviations for non-linear inner solver used during robust estimation.
     */
    protected double[] innerDistanceStandardDeviations;

    /**
     * Constructor.
     */
    protected RobustLateration3DSolver() {
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustLateration3DSolver(final RobustLaterationSolverListener<Point3D> listener) {
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
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    protected RobustLateration3DSolver(final Point3D[] positions, final double[] distances) {
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
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    protected RobustLateration3DSolver(final Point3D[] positions, final double[] distances,
                                       final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
        init();
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener  listener to be notified of events such as when estimation starts,
     *                  ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    protected RobustLateration3DSolver(final Point3D[] positions, final double[] distances,
                                       final RobustLaterationSolverListener<Point3D> listener) {
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
     *                                  standard deviations are null, don't have the same length or their length is
     *                                  smaller than required (4 points).
     */
    protected RobustLateration3DSolver(final Point3D[] positions, final double[] distances,
                                       final double[] distanceStandardDeviations,
                                       final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     *                                  is less than required (4 points).
     */
    protected RobustLateration3DSolver(final Sphere[] spheres) {
        this();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     *                                  than required (4 points) or don't have the same length.
     */
    protected RobustLateration3DSolver(final Sphere[] spheres, final double[] distanceStandardDeviations) {
        this();
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param spheres  spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     *                                  is less than required (4 points).
     */
    protected RobustLateration3DSolver(final Sphere[] spheres, final RobustLaterationSolverListener<Point3D> listener) {
        this(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation starts,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     *                                  than required (4 points) or don't have the same length.
     */
    protected RobustLateration3DSolver(final Sphere[] spheres, final double[] distanceStandardDeviations,
                                       final RobustLaterationSolverListener<Point3D> listener) {
        this(listener);
        internalSetSpheresAndStandardDeviations(spheres, distanceStandardDeviations);
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
     * At least 3 positions will be required.
     *
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
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

        innerPositions = new Point3D[preliminarySubsetSize];
        innerDistances = new double[preliminarySubsetSize];
        innerDistanceStandardDeviations = new double[preliminarySubsetSize];
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
     * @throws IllegalArgumentException if sphere is null or length of array of spheres
     *                                  is less than 4.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setSpheres(final Sphere[] spheres) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheres(spheres);
    }

    /**
     * Sets spheres defining positions and Euclidean distances along with the standard
     * deviations of provided spheres radii.
     *
     * @param spheres                  spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     *                                  4 or don't have the same length.
     * @throws LockedException          if instance is busy solving the lateration problem.
     */
    public void setSpheresAndStandardDeviations(
            final Sphere[] spheres, final double[] radiusStandardDeviations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheresAndStandardDeviations(spheres, radiusStandardDeviations);
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param method robust estimator method.
     * @return a new robust 3D lateration solver.
     */
    public static RobustLateration3DSolver create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver();
            case LMEDS -> new LMedSRobustLateration3DSolver();
            case MSAC -> new MSACRobustLateration3DSolver();
            case PROSAC -> new PROSACRobustLateration3DSolver();
            default -> new PROMedSRobustLateration3DSolver();
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a new robust 3D lateration solver.
     */
    public static RobustLateration3DSolver create(
            final RobustLaterationSolverListener<Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(listener);
            case MSAC -> new MSACRobustLateration3DSolver(listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(listener);
            default -> new PROMedSRobustLateration3DSolver(listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param method    robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances);
            case PROSAC -> new PROSACRobustLateration3DSolver(positions, distances);
            default -> new PROMedSRobustLateration3DSolver(positions, distances);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (4 points).
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case PROSAC -> new PROSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            default -> new PROMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener  listener to be notified of events such as when estimation starts,
     *                  ends or its progress significantly changes.
     * @param method    robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final RobustLaterationSolverListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, listener);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(positions, distances, listener);
            default -> new PROMedSRobustLateration3DSolver(positions, distances, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations,
                    listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations, listener);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations,
                    listener);
            default -> new PROMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param spheres spheres defining positions and distances.
     * @param method  robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(final Sphere[] spheres, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres);
            case MSAC -> new MSACRobustLateration3DSolver(spheres);
            case PROSAC -> new PROSACRobustLateration3DSolver(spheres);
            default -> new PROMedSRobustLateration3DSolver(spheres);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final double[] distanceStandardDeviations, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case MSAC -> new MSACRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case PROSAC -> new PROSACRobustLateration3DSolver(spheres, distanceStandardDeviations);
            default -> new PROMedSRobustLateration3DSolver(spheres, distanceStandardDeviations);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param spheres  spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null or if length of spheres
     *                                  array is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final RobustLaterationSolverListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres, listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres, listener);
            case MSAC -> new MSACRobustLateration3DSolver(spheres, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(spheres, listener);
            default -> new PROMedSRobustLateration3DSolver(spheres, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case MSAC -> new MSACRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            default -> new PROMedSRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param method        robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (4 samples).
     */
    public static RobustLateration3DSolver create(final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver();
            case LMEDS -> new LMedSRobustLateration3DSolver();
            case MSAC -> new MSACRobustLateration3DSolver();
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores);
            default -> new PROMedSRobustLateration3DSolver(qualityScores);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (4 samples).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final RobustLaterationSolverListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(listener);
            case MSAC -> new MSACRobustLateration3DSolver(listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, listener);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node to be
     *                      estimated.
     * @param method        robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     *                                  scores are null, don't have the same length or their length is smaller than
     *                                  required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, positions, distances);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, positions, distances);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
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
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, positions, distances,
                    distanceStandardDeviations);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, positions, distances,
                    distanceStandardDeviations);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
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
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or standard
     *                                  deviations are null, don't have the same length or their length is smaller
     *                                  than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustLaterationSolverListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations,
                    listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, distanceStandardDeviations, listener);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, distanceStandardDeviations, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, positions, distances,
                    distanceStandardDeviations, listener);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, positions, distances,
                    distanceStandardDeviations, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(positions, distances, listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(positions, distances, listener);
            case MSAC -> new MSACRobustLateration3DSolver(positions, distances, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, positions, distances, listener);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, positions, distances, listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres       spheres defining positions and distances.
     * @param method        robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres or quality scores are
     *                                  null don't have the same length or their length is less than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Sphere[] spheres, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres);
            case MSAC -> new MSACRobustLateration3DSolver(spheres);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, spheres);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, spheres);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case MSAC -> new MSACRobustLateration3DSolver(spheres, distanceStandardDeviations);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, spheres, distanceStandardDeviations);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, spheres, distanceStandardDeviations);
        };
    }

    /**
     * Creates a robust 3D lateration solver.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @param method                     robust estimator method.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case LMEDS -> new LMedSRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case MSAC -> new MSACRobustLateration3DSolver(spheres, distanceStandardDeviations, listener);
            case PROSAC -> new PROSACRobustLateration3DSolver(qualityScores, spheres, distanceStandardDeviations,
                    listener);
            default -> new PROMedSRobustLateration3DSolver(qualityScores, spheres, distanceStandardDeviations,
                    listener);
        };
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @return a new robust 3D lateration solver.
     */
    public static RobustLateration3DSolver create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     */
    public static RobustLateration3DSolver create(final RobustLaterationSolverListener<Point3D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(final Point3D[] positions, final double[] distances) {
        return create(positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations if either positions or distances are null,
     *                                   don't have the same length or their length
     *                                   is smaller than required (4 points).
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations) {
        return create(positions, distances, distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener) {
        return create(positions, distances, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        return create(positions, distances, distanceStandardDeviations, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param spheres spheres defining positions and distances.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(final Sphere[] spheres) {
        return create(spheres, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final double[] distanceStandardDeviations) {
        return create(spheres, distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param spheres  spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null or if length of spheres
     *                                  array is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final RobustLaterationSolverListener<Point3D> listener) {
        return create(spheres, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if spheres is null, length of spheres array
     *                                  is less than required (4 points) or don't have the same length.
     */
    public static RobustLateration3DSolver create(
            final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        return create(spheres, distanceStandardDeviations, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (4 samples).
     */
    public static RobustLateration3DSolver create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, enda or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if quality scores is null, length of
     *                                  quality scores is less than required minimum (4 samples).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final RobustLaterationSolverListener<Point3D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node to be
     *                      estimated.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances or quality
     *                                  scores are null, don't have the same length or their length is smaller
     *                                  than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances) {
        return create(qualityScores, positions, distances, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        return create(qualityScores, positions, distances, distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
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
     *                                  than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustLaterationSolverListener<Point3D> listener) {
        return create(qualityScores, positions, distances, distanceStandardDeviations, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either positions, distances, quality
     *                                  scores or standard deviations are null, don't have the same length or their
     *                                  length is smaller than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener) {
        return create(qualityScores, positions, distances, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param spheres       spheres defining positions and distances.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres or quality scores are
     *                                  null, don't have the same length or their length is less than required
     *                                  (4 points).
     */
    public static RobustLateration3DSolver create(final double[] qualityScores, final Sphere[] spheres) {
        return create(qualityScores, spheres, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations) {
        return create(qualityScores, spheres, distanceStandardDeviations, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D lateration solver using default robust method.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured
     *                                   distances.
     * @param listener                   listener to be notified of events such as when estimation
     *                                   starts, ends or its progress significantly changes.
     * @return a new robust 3D lateration solver.
     * @throws IllegalArgumentException if either spheres, quality scores or
     *                                  standard deviations are null, don't have the same length or their length
     *                                  is less than required (4 points).
     */
    public static RobustLateration3DSolver create(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        return create(qualityScores, spheres, distanceStandardDeviations, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Attempts to refine estimated position if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled, and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     *
     * @param position position estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or provided non-refined
     * estimated position if not requested or refinement failed.
     */
    protected Point3D attemptRefine(final Point3D position) {
        if (refineResult && inliersData != null) {
            final var inliers = inliersData.getInliers();
            final var nSamples = distances.length;
            final var nInliers = inliersData.getNumInliers();
            final var inlierPositions = new Point3D[nInliers];
            final var inlierDistances = new double[nInliers];
            double[] inlierStandardDeviations = null;
            if (distanceStandardDeviations != null) {
                inlierStandardDeviations = new double[nInliers];
            }
            var pos = 0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierPositions[pos] = positions[i];
                    inlierDistances[pos] = distances[i];
                    if (inlierStandardDeviations != null) {
                        inlierStandardDeviations[pos] = distanceStandardDeviations[i];
                    }
                    pos++;
                }
            }

            try {
                nonLinearSolver.setInitialPosition(position);
                if (inlierStandardDeviations != null) {
                    nonLinearSolver.setPositionsDistancesAndStandardDeviations(
                            inlierPositions, inlierDistances, inlierStandardDeviations);
                } else {
                    nonLinearSolver.setPositionsAndDistances(inlierPositions, inlierDistances);
                }
                nonLinearSolver.solve();

                if (keepCovariance) {
                    // keep covariance
                    covariance = nonLinearSolver.getCovariance();
                } else {
                    covariance = null;
                }

                estimatedPosition = nonLinearSolver.getEstimatedPosition();
            } catch (Exception e) {
                // refinement failed, so we return input value
                covariance = null;
                estimatedPosition = position;
            }
        } else {
            covariance = null;
            estimatedPosition = position;
        }

        return estimatedPosition;
    }

    /**
     * Solves a preliminary solution for a subset of samples picked by a robust estimator.
     *
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions      list where estimated preliminary solution will be stored.
     */
    protected void solvePreliminarySolutions(final int[] samplesIndices, final List<Point3D> solutions) {
        try {
            final var length = samplesIndices.length;
            int index;
            for (var i = 0; i < length; i++) {
                index = samplesIndices[i];
                innerPositions[i] = positions[index];
                innerDistances[i] = distances[index];
                innerDistanceStandardDeviations[i] = distanceStandardDeviations != null
                        ? distanceStandardDeviations[index]
                        : NonLinearLeastSquaresLaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;
            }

            Point3D estimatedPosition = initialPosition;
            if (useLinearSolver) {
                if (useHomogeneousLinearSolver) {
                    homogeneousLinearSolver.setPositionsAndDistances(innerPositions, innerDistances);
                    homogeneousLinearSolver.solve();
                    estimatedPosition = homogeneousLinearSolver.getEstimatedPosition();
                } else {
                    inhomogeneousLinearSolver.setPositionsAndDistances(innerPositions, innerDistances);
                    inhomogeneousLinearSolver.solve();
                    estimatedPosition = inhomogeneousLinearSolver.getEstimatedPosition();
                }
            }

            if (refinePreliminarySolutions || estimatedPosition == null) {
                nonLinearSolver.setInitialPosition(estimatedPosition);
                if (distanceStandardDeviations != null) {
                    nonLinearSolver.setPositionsDistancesAndStandardDeviations(innerPositions, innerDistances,
                            innerDistanceStandardDeviations);
                } else {
                    nonLinearSolver.setPositionsAndDistances(innerPositions, innerDistances);
                }
                nonLinearSolver.solve();
                estimatedPosition = nonLinearSolver.getEstimatedPosition();
            }

            solutions.add(estimatedPosition);
        } catch (final NavigationException ignore) {
            // if anything fails, no solution is added
        }
    }

    /**
     * Internally sets spheres defining positions and Euclidean distances.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     *                                  is less than {@link #getMinRequiredPositionsAndDistances()}
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

    /**
     * Internally sets spheres defining positions and Euclidean distances along
     * with the standard deviations of provided spheres radii.
     *
     * @param spheres                  spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     *                                  4 or don't have the same length.
     */
    private void internalSetSpheresAndStandardDeviations(
            final Sphere[] spheres, final double[] radiusStandardDeviations) {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != spheres.length) {
            throw new IllegalArgumentException();
        }

        final var positions = new Point3D[spheres.length];
        final var distances = new double[spheres.length];
        for (var i = 0; i < spheres.length; i++) {
            final var sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances, radiusStandardDeviations);
    }

    /**
     * Setup inner positions and distances.
     */
    private void init() {
        final int points = getMinRequiredPositionsAndDistances();
        preliminarySubsetSize = points;
        innerPositions = new Point3D[points];
        innerDistances = new double[points];
        innerDistanceStandardDeviations = new double[points];

        inhomogeneousLinearSolver = new InhomogeneousLinearLeastSquaresLateration3DSolver();
        homogeneousLinearSolver = new HomogeneousLinearLeastSquaresLateration3DSolver();
        nonLinearSolver = new NonLinearLeastSquaresLateration3DSolver();
    }
}
