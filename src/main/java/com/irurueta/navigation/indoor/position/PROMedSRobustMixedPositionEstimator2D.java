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
import com.irurueta.navigation.lateration.PROMedSRobustLateration2DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 2D position using located radio sources and their
 * ranging+RSSI readings at unknown locations and using PROMedS algorithm to discard
 * outliers.
 * This kind of estimator can be used to robustly determine the 2D position of a given
 * device by getting ranging+RSSI readings at an unknown location of different radio
 * sources whose 2D locations are known.
 */
@SuppressWarnings("WeakerAccess")
public class PROMedSRobustMixedPositionEstimator2D extends
        RobustMixedPositionEstimator2D {

    /**
     * Quality scores corresponding to each provided located radio source.
     * The larger the score value the better the quality of the radio source.
     */
    private double[] mSourceQualityScores;

    /**
     * Quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     */
    private double[] mFingerprintReadingsQualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustMixedPositionEstimator2D() {
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
    public PROMedSRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources) {
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
    public PROMedSRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
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
    public PROMedSRobustMixedPositionEstimator2D(
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
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
    public PROMedSRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
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
    public PROMedSRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
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
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores) {
        this();
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param sources                          located radio sources used for
     *                                         lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        this(sources);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param fingerprint                      fingerprint containing ranging+RSSI
     *                                         readings at an unknown location for
     *                                         provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param sources                          located radio sources used for
     *                                         lateration.
     * @param fingerprint                      fingerprint containing ranging+RSSI
     *                                         readings at an unknown location for
     *                                         provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(sources, fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param listener                         listener in charge of handling events.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(listener);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param sources                          located radio sources used for
     *                                         lateration.
     * @param listener                         listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(sources, listener);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param fingerprint                      fingerprint containing ranging+RSSI
     *                                         readings at an unknown location for
     *                                         provided location radio sources.
     * @param listener                         listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(fingerprint, listener);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores              quality scores corresponding to
     *                                         each provided located radio source.
     *                                         The larger the score value the better
     *                                         the quality of the radio source.
     * @param fingerprintReadingsQualityScores quality scores corresponding to readings
     *                                         within provided fingerprint. The larger
     *                                         the score the better the quality of the
     *                                         reading.
     * @param sources                          located radio sources used for
     *                                         lateration.
     * @param fingerprint                      fingerprint containing ranging+RSSI
     *                                         readings at an unknown location for
     *                                         provided located radio sources.
     * @param listener                         listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less than the required minimum.
     */
    public PROMedSRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingsQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(sources, fingerprint, listener);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each radio source.
     */
    public double[] getSourceQualityScores() {
        return mSourceQualityScores;
    }

    /**
     * Sets quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the radio source.
     *
     * @param sourceQualityScores quality scores corresponding to each radio source.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setSourceQualityScores(final double[] sourceQualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSourceQualityScores(sourceQualityScores);
    }

    /**
     * Gets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each reading within provided
     * fingerprint.
     */
    public double[] getFingerprintReadingsQualityScores() {
        return mFingerprintReadingsQualityScores;
    }

    /**
     * Sets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @param fingerprintReadingsQualityScores quality scores corresponding to each
     *                                         reading within provided fingerprint.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setFingerprintReadingsQualityScores(
            final double[] fingerprintReadingsQualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
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
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return ((PROMedSRobustLateration2DSolver) mLaterationSolver).
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
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if this solver is locked.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        ((PROMedSRobustLateration2DSolver) mLaterationSolver).
                setStopThreshold(stopThreshold);
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMedS;
    }

    /**
     * Initializes robust lateration solver.
     */
    private void init() {
        mLaterationSolver = new PROMedSRobustLateration2DSolver(
                mTrilaterationSolverListener);
    }

    /**
     * Sets quality scores corresponding to each provided located radio source.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param sourceQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples.
     */
    private void internalSetSourceQualityScores(final double[] sourceQualityScores) {
        if (sourceQualityScores == null ||
                sourceQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mSourceQualityScores = sourceQualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Sets quality scores corresponding to each provided reading within provided
     * fingerprint.
     * This method is used internally and does not check whether instance is locked
     * or not.
     *
     * @param fingerprintReadingsQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores lengt is
     *                                  smaller than 3 samples.
     */
    private void internalSetFingerprintReadingsQualityScores(
            final double[] fingerprintReadingsQualityScores) {
        if (fingerprintReadingsQualityScores == null ||
                fingerprintReadingsQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mFingerprintReadingsQualityScores = fingerprintReadingsQualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }
}
