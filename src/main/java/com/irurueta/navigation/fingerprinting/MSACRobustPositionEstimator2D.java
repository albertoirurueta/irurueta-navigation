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
package com.irurueta.navigation.fingerprinting;

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.trilateration.MSACRobustTrilateration2DSolver;

import java.util.List;

/**
 * Robustly estimates 2D position using located radio sources and their
 * readings at unknown locations and using MSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 2D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 2D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class MSACRobustPositionEstimator2D extends RobustPositionEstimator2D {

    /**
     * Constructor.
     */
    public MSACRobustPositionEstimator2D() {
        super();
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public MSACRobustPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources)
            throws IllegalArgumentException {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustPositionEstimator2D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public MSACRobustPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        super();
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public MSACRobustPositionEstimator2D(
            RobustPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public MSACRobustPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustPositionEstimatorListener<Point2D> listener)
            throws IllegalArgumentException {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location for provided
     *                    location radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public MSACRobustPositionEstimator2D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) throws IllegalArgumentException {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     * null or the number of provided sources is less than the required minimum.
     */
    public MSACRobustPositionEstimator2D(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) throws IllegalArgumentException {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return ((MSACRobustTrilateration2DSolver)mTrilaterationSolver).
                getThreshold();
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException if this solver is locked.
     */
    public void setThreshold(double threshold)
            throws IllegalArgumentException, LockedException {
        ((MSACRobustTrilateration2DSolver)mTrilaterationSolver).
                setThreshold(threshold);
    }

    /**
     * Initializes robust trilateration solver.
     */
    private void init() {
        mTrilaterationSolver = new MSACRobustTrilateration2DSolver(
                mTrilaterationSolverListener);
    }
}
