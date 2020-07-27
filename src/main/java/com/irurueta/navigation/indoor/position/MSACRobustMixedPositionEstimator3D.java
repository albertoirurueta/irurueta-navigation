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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.lateration.MSACRobustLateration3DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 3D position using located radio sources and their readings at
 * unknown locations and using MSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 3D position of a given
 * device by getting readings at an unknown location of different radio
 * sources whose 3D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class MSACRobustMixedPositionEstimator3D extends
        RobustMixedPositionEstimator3D {

    /**
     * Constructor.
     */
    public MSACRobustMixedPositionEstimator3D() {
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
    public MSACRobustMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustMixedPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public MSACRobustMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public MSACRobustMixedPositionEstimator3D(
            final RobustMixedPositionEstimatorListener<Point3D> listener) {
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
    public MSACRobustMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustMixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided location radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustMixedPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     */
    public MSACRobustMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point3D> listener) {
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
        return ((MSACRobustLateration3DSolver) mLaterationSolver).
                getThreshold();
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
        ((MSACRobustLateration3DSolver) mLaterationSolver).
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
        mLaterationSolver = new MSACRobustLateration3DSolver(
                mTrilaterationSolverListener);
    }
}
