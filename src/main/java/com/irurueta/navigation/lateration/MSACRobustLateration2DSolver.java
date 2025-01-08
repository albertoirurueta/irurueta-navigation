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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly solves the lateration problem by finding the best pairs of 2D
 * positions and distances among the provided ones using MSAC algorithm to
 * discard outliers.
 */
@SuppressWarnings("Duplicates")
public class MSACRobustLateration2DSolver extends RobustLateration2DSolver {

    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-2;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether samples are inliers or not when
     * testing possible estimation solutions.
     */
    private double threshold = DEFAULT_THRESHOLD;

    /**
     * Constructor.
     */
    public MSACRobustLateration2DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACRobustLateration2DSolver(final RobustLaterationSolverListener<Point2D> listener) {
        super(listener);
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
    public MSACRobustLateration2DSolver(final Point2D[] positions, final double[] distances) {
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
     *                                  don't have the same length or their length is smaller than required (3 points).
     */
    public MSACRobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                        final double[] distanceStandardDeviations) {
        super(positions, distances, distanceStandardDeviations);
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
     *                                  smaller than required (3 points).
     */
    public MSACRobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                        final double[] distanceStandardDeviations,
                                        final RobustLaterationSolverListener<Point2D> listener) {
        super(positions, distances, distanceStandardDeviations, listener);
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
    public MSACRobustLateration2DSolver(final Point2D[] positions, final double[] distances,
                                        final RobustLaterationSolverListener<Point2D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     *                                  is less than required (3 points).
     */
    public MSACRobustLateration2DSolver(final Circle[] circles) {
        super(circles);
    }

    /**
     * Constructor.
     *
     * @param circles                    circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     *                                  than required (3 points) or don't have the same length.
     */
    public MSACRobustLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations) {
        super(circles, distanceStandardDeviations);
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
    public MSACRobustLateration2DSolver(
            final Circle[] circles, final RobustLaterationSolverListener<Point2D> listener) {
        super(circles, listener);
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
    public MSACRobustLateration2DSolver(
            final Circle[] circles, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point2D> listener) {
        super(circles, distanceStandardDeviations, listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
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
     * Solves the lateration problem.
     *
     * @return estimated position.
     * @throws LockedException          if instance is busy solving the lateration problem.
     * @throws NotReadyException        is solver is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point2D solve() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new MSACRobustEstimator<>(new MSACRobustEstimatorListener<Point2D>() {
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
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Point2D> solutions) {
                solvePreliminarySolutions(samplesIndices, solutions);
            }

            @Override
            public double computeResidual(final Point2D currentEstimation, final int i) {
                return Math.abs(currentEstimation.distanceTo(positions[i]) - distances[i]);
            }

            @Override
            public boolean isReady() {
                return MSACRobustLateration2DSolver.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Point2D> estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Point2D> estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Point2D> estimator, final int iteration) {
                if (listener != null) {
                    listener.onSolveNextIteration(MSACRobustLateration2DSolver.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Point2D> estimator, final float progress) {
                if (listener != null) {
                    listener.onSolveProgressChange(MSACRobustLateration2DSolver.this, progress);
                }
            }
        });

        try {
            locked = true;

            if (listener != null) {
                listener.onSolveStart(this);
            }

            inliersData = null;
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
        return RobustEstimatorMethod.MSAC;
    }
}
