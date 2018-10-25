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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly solves the trilateration problem by finding the best pairs of 3D
 * positions and distances among the provided ones using MSAC algorithm to
 * discard outliers.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class MSACRobustTrilateration3DSolver extends RobustTrilateration3DSolver {

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
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Constructor.
     */
    public MSACRobustTrilateration3DSolver() {
        super();
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACRobustTrilateration3DSolver(
            RobustTrilaterationSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public MSACRobustTrilateration3DSolver(Point3D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
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
    public MSACRobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
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
    public MSACRobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public MSACRobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public MSACRobustTrilateration3DSolver(Sphere[] spheres)
            throws IllegalArgumentException {
        super(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     * than required (4 points) or don't have the same length.
     */
    public MSACRobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(spheres, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public MSACRobustTrilateration3DSolver(Sphere[] spheres,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(spheres, listener);
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
    public MSACRobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(spheres, distanceStandardDeviations, listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     * zero.
     * @throws LockedException if robust estimator is locked because an
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Solves the trilateration problem.
     * @return estimated position.
     * @throws LockedException if instance is busy solving the trilateration problem.
     * @throws NotReadyException is solver is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point3D solve() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        MSACRobustEstimator<Point3D> innerEstimator =
            new MSACRobustEstimator<>(new MSACRobustEstimatorListener<Point3D>() {
                @Override
                public double getThreshold() {
                    return mThreshold;
                }

                @Override
                public int getTotalSamples() {
                    return mDistances.length;
                }

                @Override
                public int getSubsetSize() {
                    return getMinRequiredPositionsAndDistances();
                }

                @Override
                public void estimatePreliminarSolutions(int[] samplesIndices, List<Point3D> solutions) {
                    solvePreliminarSolutions(samplesIndices, solutions);
                }

                @Override
                public double computeResidual(Point3D currentEstimation, int i) {
                    return Math.abs(currentEstimation.distanceTo(mPositions[i]) - mDistances[i]);
                }

                @Override
                public boolean isReady() {
                    return MSACRobustTrilateration3DSolver.this.isReady();
                }

                @Override
                public void onEstimateStart(RobustEstimator<Point3D> estimator) {
                    if (mListener != null) {
                        mListener.onSolveStart(MSACRobustTrilateration3DSolver.this);
                    }
                }

                @Override
                public void onEstimateEnd(RobustEstimator<Point3D> estimator) {
                    if (mListener != null) {
                        mListener.onSolveEnd(MSACRobustTrilateration3DSolver.this);
                    }
                }

                @Override
                public void onEstimateNextIteration(RobustEstimator<Point3D> estimator, int iteration) {
                    if (mListener != null) {
                        mListener.onSolveNextIteration(MSACRobustTrilateration3DSolver.this,
                                iteration);
                    }
                }

                @Override
                public void onEstimateProgressChange(RobustEstimator<Point3D> estimator, float progress) {
                    if (mListener != null) {
                        mListener.onSolveProgressChange(MSACRobustTrilateration3DSolver.this,
                                progress);
                    }
                }
            });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Point3D result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }
}
