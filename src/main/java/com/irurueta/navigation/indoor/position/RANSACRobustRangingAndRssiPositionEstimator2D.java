/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.lateration.RANSACRobustLateration2DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 2D position using located ranging+RSSI radio sources and their
 * ranging readings at unknown locations and using RANSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 2D position of a given
 * device by getting ranging+RSSI readings at an unknown location of different radio
 * sources whose 2D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class RANSACRobustRangingAndRssiPositionEstimator2D extends
        RobustRangingAndRssiPositionEstimator2D {

    /**
     * Constructor.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint) {
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
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final RobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint,
            final RobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less than the required minimum.
     */
    public RANSACRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint,
            final RobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
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
        return ((RANSACRobustLateration2DSolver) mLaterationSolver).
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
    public void setThreshold(final double threshold) throws LockedException {
        ((RANSACRobustLateration2DSolver) mLaterationSolver).
                setThreshold(threshold);
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return ((RANSACRobustLateration2DSolver) mLaterationSolver).
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
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers)
            throws LockedException {
        ((RANSACRobustLateration2DSolver) mLaterationSolver).
                setComputeAndKeepInliersEnabled(computeAndKeepInliers);
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return ((RANSACRobustLateration2DSolver) mLaterationSolver).
                isComputeAndKeepResiduals();
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals)
            throws LockedException {
        ((RANSACRobustLateration2DSolver) mLaterationSolver).
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
     * Initializes robust lateration solver.
     */
    private void init() {
        mLaterationSolver = new RANSACRobustLateration2DSolver(
                mTrilaterationSolverListener);
    }
}
