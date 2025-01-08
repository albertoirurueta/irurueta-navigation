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
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly solves the lateration problem by finding the best pairs of 3D
 * positions and distances among the provided ones using LMedS algorithm to
 * discard outliers.
 */
@SuppressWarnings("Duplicates")
public class LMedSRobustLateration3DSolver extends RobustLateration3DSolver {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-5;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double stopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Constructor.
     */
    public LMedSRobustLateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public LMedSRobustLateration3DSolver(final RobustLaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    public LMedSRobustLateration3DSolver(final Point3D[] positions, final double[] distances) {
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
    public LMedSRobustLateration3DSolver(final Point3D[] positions, final double[] distances,
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
     *                                  smaller than required (4 points).
     */
    public LMedSRobustLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
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
     *                                  don't have the same length or their length is smaller than required (4 points).
     */
    public LMedSRobustLateration3DSolver(
            final Point3D[] positions, final double[] distances,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     *                                  is less than required (4 points).
     */
    public LMedSRobustLateration3DSolver(final Sphere[] spheres) {
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
    public LMedSRobustLateration3DSolver(final Sphere[] spheres,
                                         final double[] distanceStandardDeviations) {
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
    public LMedSRobustLateration3DSolver(
            final Sphere[] spheres, final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, listener);
    }

    /**
     * Constructor.
     *
     * @param spheres                    spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener                   listener to be notified of events such as when estimation stats,
     *                                   ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     *                                  than required (4 points) or don't have the same length.
     */
    public LMedSRobustLateration3DSolver(
            final Sphere[] spheres, final double[] distanceStandardDeviations,
            final RobustLaterationSolverListener<Point3D> listener) {
        super(spheres, distanceStandardDeviations, listener);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return stopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if this solver is locked.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.stopThreshold = stopThreshold;
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

        final var innerEstimator = new LMedSRobustEstimator<>(new LMedSRobustEstimatorListener<Point3D>() {
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
            public double computeResidual(final Point3D currentEstimation, final int i) {
                return Math.abs(currentEstimation.distanceTo(positions[i]) - distances[i]);
            }

            @Override
            public boolean isReady() {
                return LMedSRobustLateration3DSolver.this.isReady();
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
                    listener.onSolveNextIteration(LMedSRobustLateration3DSolver.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Point3D> estimator, final float progress) {
                if (listener != null) {
                    listener.onSolveProgressChange(LMedSRobustLateration3DSolver.this, progress);
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
        return RobustEstimatorMethod.LMEDS;
    }
}
