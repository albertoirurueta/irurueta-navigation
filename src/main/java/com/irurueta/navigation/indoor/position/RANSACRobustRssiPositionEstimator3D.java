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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.trilateration.RANSACRobustTrilateration3DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 3D position using located RSSI radio sources and their
 * RSSI readings at unknown locations and using RANSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 3D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 3D locations are known.
 */
public class RANSACRobustRssiPositionEstimator3D extends RobustRssiPositionEstimator3D {

    /**
     * Constructor.
     */
    public RANSACRobustRssiPositionEstimator3D() {
        super();
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public RANSACRobustRssiPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public RANSACRobustRssiPositionEstimator3D(
            RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for trilateration.
     * @param fingerprint   fingerprint containing readings at an unknown location for
     *                      provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public RANSACRobustRssiPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RANSACRobustRssiPositionEstimator3D(
            RobustRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources   located radio sources used for trilateration.
     * @param listener  listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public RANSACRobustRssiPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            RobustRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint   fingerprint containing RSSI readings at an unknown location
     *                      for provided located radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public RANSACRobustRssiPositionEstimator3D(
            Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            RobustRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for trilateration.
     * @param fingerprint   fingerprint containing readings at an unknown location for
     *                      provided located radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     * null or the number of provided sources is less than the required minimum.
     */
    public RANSACRobustRssiPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            RobustRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing
     * possible solutions.
     * The threshold refers to the amount of error on distance between estimated
     * position and distances provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                getThreshold();
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing
     * possible solutions.
     * The threshold refers to the amount of error on distance between estimated position
     * and distances provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if this estimator is locked.
     */
    public void setThreshold(double threshold) throws LockedException {
        ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setThreshold(threshold);
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                isComputeAndKeepInliersEnabled();
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not
     *                              kept.
     * @throws LockedException if this estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers)
            throws LockedException {
        ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setComputeAndKeepInliersEnabled(computeAndKeepInliers);
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                isComputeAndKeepResiduals();
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(boolean computeAndKeepResiduals)
            throws LockedException {
        ((RANSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setComputeAndKeepResidualsEnabled(computeAndKeepResiduals);
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.RANSAC;
    }

    /**
     * Initializes robust trilateration solver.
     */
    private void init() {
        mTrilaterationSolver = new RANSACRobustTrilateration3DSolver(
                mTrilaterationSolverListener);
    }
}
