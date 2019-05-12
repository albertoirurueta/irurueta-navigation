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
import com.irurueta.navigation.lateration.MSACRobustLateration2DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 2D position using located radio sources and their
 * RSSI readings at unknown locations and using MSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 2D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 2D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class MSACRobustRangingPositionEstimator2D extends
        RobustRangingPositionEstimator2D {

    /**
     * Constructor.
     */
    public MSACRobustRangingPositionEstimator2D() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public MSACRobustRangingPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustRangingPositionEstimator2D(
            RangingFingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing ranging readings at an unknown location
     *                      for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public MSACRobustRangingPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RangingFingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint) {
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
    public MSACRobustRangingPositionEstimator2D(
            RobustRangingPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources   located radio sources used for lateration.
     * @param listener  listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public MSACRobustRangingPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustRangingPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint   fingerprint containing ranging readings at an unknown
     *                      location for provided location radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustRangingPositionEstimator2D(
            RangingFingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            RobustRangingPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing ranging readings at an unknown
     *                      location for provided located radio sources.
     * @param listener      listener in charge of handling events.
     */
    public MSACRobustRangingPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RangingFingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            RobustRangingPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return ((MSACRobustLateration2DSolver) mLaterationSolver).
                getThreshold();
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException if this solver is locked.
     */
    public void setThreshold(double threshold) throws LockedException {
        ((MSACRobustLateration2DSolver) mLaterationSolver).
                setThreshold(threshold);
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

    /**
     * Initializes robust lateration solver.
     */
    private void init() {
        mLaterationSolver = new MSACRobustLateration2DSolver(
                mTrilaterationSolverListener);
    }
}
