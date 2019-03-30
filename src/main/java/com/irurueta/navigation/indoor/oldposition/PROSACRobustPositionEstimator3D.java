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
package com.irurueta.navigation.indoor.oldposition;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.trilateration.PROSACRobustTrilateration3DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 3D position using located radio sources and their
 * readings at unknown locations and using PROSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 3D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 3D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class PROSACRobustPositionEstimator3D extends RobustPositionEstimator3D {

    /**
     * Quality scores corresponding to each provided located radio source.
     * The larger the score value the better the quality of the radio source.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustPositionEstimator3D() {
        super();
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public PROSACRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources) {
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
    public PROSACRobustPositionEstimator3D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public PROSACRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public PROSACRobustPositionEstimator3D(
            RobustPositionEstimatorListener<Point3D> listener) {
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
    public PROSACRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            RobustPositionEstimatorListener<Point3D> listener) {
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
    public PROSACRobustPositionEstimator3D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) {
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
    public PROSACRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores) {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources) {
        this(sources);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(fingerprint);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(sources, fingerprint);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param listener listener in charge of handling events.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            RobustPositionEstimatorListener<Point3D> listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            RobustPositionEstimatorListener<Point3D> listener) {
        this(sources, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param fingerprint fingerprint containing readings at an unknown location for provided
     *                    location radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) {
        this(fingerprint, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                      value the better the quality of the radio
     *                      source.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     * null or the number of provided sources is less than the required minimum.
     */
    public PROSACRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) {
        this(sources, fingerprint, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns quality scores corresponding to each provided located
     * radio source.
     * The larger the score value the better the quality of the radio
     * source.
     * @return quality scores corresponding to each radio source.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided located
     * radio source.
     * The larger the score value the better the quality of the radio
     * source.
     * @param qualityScores quality scores corresponding to each radio
     *                      source.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
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
    public void setThreshold(double threshold) throws LockedException {
        ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setThreshold(threshold);
    }

    /**
     * Indicates whether inliers must be computed and kept.
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
                isComputeAndKeepInliersEnabled();
    }

    /**
     * Specifies whether inliers must be computed and kept.
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers)
            throws LockedException {
        ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setComputeAndKeepInliersEnabled(computeAndKeepInliers);
    }

    /**
     * Indicates whether residuals must be computed and kept.
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
                isComputeAndKeepResiduals();
    }

    /**
     * Specifies whether residuals must be computed and kept.
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepResidualsEnabled(boolean computeAndKeepResiduals)
            throws LockedException {
        ((PROSACRobustTrilateration3DSolver)mTrilaterationSolver).
                setComputeAndKeepResidualsEnabled(computeAndKeepResiduals);
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Initializes robust trilateration solver.
     */
    private void init() {
        mTrilaterationSolver = new PROSACRobustTrilateration3DSolver(
                mTrilaterationSolverListener);
    }

    /**
     * Sets quality scores corresponding to each provided located radio source.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than 3 samples.
     */
    private void internalSetQualityScores(double[] qualityScores) {
        if (qualityScores == null ||
                qualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }
}
