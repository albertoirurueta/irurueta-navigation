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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.trilateration.PROMedSRobustTrilateration3DSolver;

import java.util.List;

/**
 * Robustly estimates 3D position using located radio sources and their
 * readings at unknown locations and using PROMedS algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 3D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 3D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class PROMedSRobustPositionEstimator3D extends RobustPositionEstimator3D {

    /**
     * Quality scores corresponding to each provided located radio source.
     * The larger the score value the better the quality of the radio source.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustPositionEstimator3D() {
        super();
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public PROMedSRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources)
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
    public PROMedSRobustPositionEstimator3D(
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
    public PROMedSRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
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
    public PROMedSRobustPositionEstimator3D(
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
    public PROMedSRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            RobustPositionEstimatorListener<Point3D> listener)
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
    public PROMedSRobustPositionEstimator3D(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores) {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources)
            throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            RobustPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) throws IllegalArgumentException {
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
    public PROMedSRobustPositionEstimator3D(double[] qualityScores,
            List<? extends RadioSourceLocated<Point3D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point3D> listener) throws IllegalArgumentException {
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
    public void setQualityScores(double[] qualityScores)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algrithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return ((PROMedSRobustTrilateration3DSolver)mTrilaterationSolver).
                getStopThreshold();
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
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if this solver is locked.
     */
    public void setStopThreshold(double stopThreshold)
            throws IllegalArgumentException, LockedException {
        ((PROMedSRobustTrilateration3DSolver)mTrilaterationSolver).
                setStopThreshold(stopThreshold);
    }

    /**
     * Initializes robust trilateration solver.
     */
    private void init() {
        mTrilaterationSolver = new PROMedSRobustTrilateration3DSolver(
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
    private void internalSetQualityScores(double[] qualityScores)
            throws IllegalArgumentException {
        if (qualityScores == null ||
                qualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }
}
