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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly solves the lateration problem by finding the best pairs of 3D
 * positions and distances among the provided ones using PROSAC algorithm to
 * discard outliers.
 */
@SuppressWarnings("Duplicates")
public class PROSACRobustLateration3DSolver extends RobustLateration3DSolver {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-2;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and
     * distances provided for each sample.
     */
    private double threshold = DEFAULT_THRESHOLD;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustLateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustLateration3DSolver(final RobustLaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length of their length is smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(final Point3D[] positions, final double[] distances) {
        super(positions, distances);
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
    public PROSACRobustLateration3DSolver(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
    }

    /**
     * Constructor.
     *
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation stats,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length is
     *                                  smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final Point3D[] positions, final double[] distances, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener  listener to be notified of events such as when estimation stats,
     *                  ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length or spheres array
     *                                  is less than required (4 points).
     */
    public PROSACRobustLateration3DSolver(final Sphere[] spheres) {
        super(spheres);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     *                                  than required (4 points) or don't have the same length.
     */
    public PROSACRobustLateration3DSolver(
            final Sphere[] spheres, final double[] distanceStandardDeviations) {
        super(spheres, distanceStandardDeviations);
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
    public PROSACRobustLateration3DSolver(
            final Sphere[] spheres, final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, listener);
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
    public PROSACRobustLateration3DSolver(
            final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @throws IllegalArgumentException if quality scores is null, length
     *                                  of quality scores is less than required minimum (4 samples).
     */
    public PROSACRobustLateration3DSolver(final double[] qualityScores) {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if quality scores is null, length
     *                                  of quality scores is less than required minimum (4 samples).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final RobustLaterationSolverListener<Point3D> listener) {
        super(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node to be
     *                      estimated.
     * @throws IllegalArgumentException if either positions, distances or quality
     *                                  scores are null, don't have the same length of their length is smaller
     *                                  than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Point3D[] positions, final double[] distances) {
        super(positions, distances);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node to be
     *                                   estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances, quality scores or
     *                                  standard deviations are null, don't have the same length or their length is
     *                                  smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param positions                  known positions of static nodes.
     * @param distances                  euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation starts,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     *                                  standard deviations are null, don't have the same length or their length is
     *                                  smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations, final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions     known positions of static nodes.
     * @param distances     euclidean distances from static nodes to mobile node.
     * @param listener      listener to be notified of events such as when
     *                      estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances,
     *                                  quality scores or standard deviations are null, don't have the same
     *                                  length or their length is smaller than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param spheres       spheres defining positions and distances.
     * @throws IllegalArgumentException if either spheres or quality scores
     *                                  are null don't have the same length or their length is less than
     *                                  required (4 points).
     */
    public PROSACRobustLateration3DSolver(final double[] qualityScores, final Sphere[] spheres) {
        super(spheres);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either spheres, quality scores or
     *                                  standard deviations are null, don't have the same length or their
     *                                  length is less than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations) {
        super(spheres, distanceStandardDeviations);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param spheres       spheres defining positions and distances.
     * @param listener      listener to be notified of events such as when estimation starts,
     *                      ends or its progress significantly changes.
     * @throws IllegalArgumentException if either spheres or quality scores
     *                                  are null, don't have the same length or their length is less than
     *                                  required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Sphere[] spheres,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation starts,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if either spheres, quality scores
     *                                  or standard deviations are null, don't have the same length or their
     *                                  length is less than required (4 points).
     */
    public PROSACRobustLateration3DSolver(
            final double[] qualityScores, final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, distanceStandardDeviations, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if this solver is locked.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of positions and
     * distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if robust solver is locked because an
     *                                  estimation is already in progress.
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == distances.length;
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return computeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return computeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Solves the lateration problem.
     *
     * @return estimated position.
     * @throws LockedException          if instance is busy solving the lateration problem.
     * @throws NotReadyException        is solver is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point3D solve() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<Point3D>() {
            @Override
            public double[] getQualityScores() {
                return qualityScores;
            }

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return distances.length;
            }

            @Override
            public int getSubsetSize() {
                return preliminarySubsetSize;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Point3D> solutions) {
                solvePreliminarySolutions(samplesIndices, solutions);
            }

            @Override
            public double computeResidual(final Point3D currentEstimation, int i) {
                return Math.abs(currentEstimation.distanceTo(positions[i]) - distances[i]);
            }

            @Override
            public boolean isReady() {
                return PROSACRobustLateration3DSolver.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Point3D> estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Point3D> estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Point3D> estimator, final int iteration) {
                if (listener != null) {
                    listener.onSolveNextIteration(PROSACRobustLateration3DSolver.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Point3D> estimator, final float progress) {
                if (listener != null) {
                    listener.onSolveProgressChange(PROSACRobustLateration3DSolver.this, progress);
                }
            }
        });

        try {
            locked = true;

            if (listener != null) {
                listener.onSolveStart(this);
            }

            inliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(computeAndKeepInliers || refineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(computeAndKeepResiduals || refineResult);
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            var result = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();
            result = attemptRefine(result);

            if (listener != null) {
                listener.onSolveEnd(this);
            }

            return result;

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null || qualityScores.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
