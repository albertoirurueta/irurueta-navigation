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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for robust position estimation, using RSSI readings first to obtain
 * an initial coarse position estimation, and then ranging readings to refine such
 * estimation.
 *
 * This implementation is like SequentialRobustRangingAndRssiPositionEstimator but
 * allows mixing different kinds of readings (ranging, RSSI or ranging+RSSI).
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class SequentialRobustMixedPositionEstimator<P extends Point> {

    /**
     * Default robust estimator method for robust position estimation using ranging
     * data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_RANGING_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Default robust method for coarse robust position estimation using RSSI
     * data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_RSSI_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that by default located radio source position covariance is taken
     * into account (if available) to determine distance standard deviation for ranging
     * measurements.
     */
    public static final boolean DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE =
            true;

    /**
     * Indicates that by default located radio source position covariance is taken
     * into account (if available) to determine distance standard deviation for RSSI
     * measurements.
     */
    public static final boolean DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE =
            true;

    /**
     * Indicates that by default readings are distributed evenly among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     */
    public static final boolean DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS = true;

    /**
     * Indicates that by default readings are distributed evenly among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     */
    public static final boolean DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS = true;

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
     * By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Indicates that result is refined by default using all found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Indicates that by default a linear solver is used for preliminary solution
     * estimation using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_RANGING_LINEAR_SOLVER = true;

    /**
     * Indicates that by default a linear solver is used for preliminary solution
     * estimation using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_RSSI_LINEAR_SOLVER = true;

    /**
     * Indicates that by default an homogeneous linear solver is used either to
     * estimate preliminary solutions or an initial solution for preliminary solutions
     * that will be later refined on the ranging fine estimation.
     */
    public static final boolean DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER = false;

    /**
     * Indicates that by default an homogeneous linear solver is used either to
     * estimate preliminary solutions or an initial solution for preliminary solutions
     * that will be later refined on the RSSI coarse estimation.
     */
    public static final boolean DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER = false;

    /**
     * Indicates that by default preliminary ranging solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS = true;

    /**
     * Indicates that by default preliminary RSSI solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS = true;

    /**
     * Internal robust estimator for position estimation using ranging readings.
     */
    protected RobustRangingPositionEstimator<P> mRangingEstimator;

    /**
     * Internal robust estimator for coarse position estimation using RSSI readings.
     */
    protected RobustRssiPositionEstimator<P> mRssiEstimator;

    /**
     * Robust method used for robust position estimation using ranging data.
     */
    protected RobustEstimatorMethod mRangingRobustMethod =
            DEFAULT_RANGING_ROBUST_METHOD;

    /**
     * Robust method used for coarse robust position estimation using RSSI data.
     */
    protected RobustEstimatorMethod mRssiRobustMethod =
            DEFAULT_RSSI_ROBUST_METHOD;

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     */
    private boolean mUseRangingRadioSourcePositionCovariance =
            DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for RSSI measurements.
     */
    private boolean mUseRssiRadioSourcePositionCovariance =
            DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Indicates whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     */
    private boolean mEvenlyDistributeRangingReadings =
            DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS;

    /**
     * Indicates whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     */
    private boolean mEvenlyDistributeRssiReadings =
            DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS;

    /**
     * Distance standard deviation fallback value to use when none can be determined
     * from provided RSSI measurements.
     */
    private double mRssiFallbackDistanceStandardDeviation =
            FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Distance standard deviation fallback value to use when none can be determined
     * from provided ranging measurements.
     */
    private double mRangingFallbackDistanceStandardDeviation =
            FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    private float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on ranging data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     */
    private double mRangingConfidence = DEFAULT_CONFIDENCE;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     */
    private double mRssiConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations for robust ranging position estimation.
     * When the maximum number of iterations is exceeded, an approximate result
     * might be available for retrieval.
     */
    private int mRangingMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result
     * might be available for retrieval.
     */
    private int mRssiMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Indicates whether result is refined using all found inliers.
     */
    private boolean mRefineResult = DEFAULT_REFINE_RESULT;

    /**
     * Indicates that covariance is kept after refining result.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Indicates that a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    private boolean mUseRangingLinearSolver = DEFAULT_USE_RANGING_LINEAR_SOLVER;

    /**
     * Indicates that a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    private boolean mUseRssiLinearSolver = DEFAULT_USE_RSSI_LINEAR_SOLVER;

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     */
    private boolean mUseRangingHomogeneousLinearSolver =
            DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     */
    private boolean mUseRssiHomogeneousLinearSolver =
            DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether preliminary ranging solutions are refined.
     */
    private boolean mRefineRangingPreliminarySolutions =
            DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS;

    /**
     * Indicates whether preliminary RSSI solutions are refined.
     */
    private boolean mRefineRssiPreliminarySolutions =
            DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS;

    /**
     * Threshold to determine when samples are inliers or not used during ranging
     * position estimation.
     * If not defined, default threshold will be used.
     */
    private Double mRangingThreshold;

    /**
     * Threshold to determine when samples are inliers or not used during RSSI
     * position estimation.
     * If not defined, default threshold will be used.
     */
    private Double mRssiThreshold;

    /**
     * Listener in charge of handling events.
     */
    private SequentialRobustMixedPositionEstimatorListener<P> mListener;

    /**
     * Located radio sources used for lateration.
     */
    private List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Fingerprint containing readings at an unknown location for provided located
     * radio sources.
     */
    private Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> mFingerprint;

    /**
     * Quality scores corresponding to each provided located radio source.
     * The larger the score value the better the quality of the radio source.
     */
    private double[] mSourceQualityScores;

    /**
     * Quality scores corresponding to each reading with provided fingerprint.
     * The larger the score value the better the quality of the reading.
     */
    private double[] mFingerprintReadingsQualityScores;

    /**
     * An initial position to start the estimation from. This can be useful if we only
     * intend to refine a previously known estimation.
     */
    private P mInitialPosition;

    /**
     * Indicates if this instance is locked because estimation is being executed.
     */
    private boolean mLocked;

    /**
     * Indicates whether ranging estimation must be available or not using
     * provided fingerprint readings.
     */
    private boolean mRangingEstimatorAvailable;

    /**
     * Indicates whether RSSI estimation must be available or not using
     * provided fingerprint readings.
     */
    private boolean mRssiEstimatorAvailable;

    /**
     * Number of ranging readings found on provided fingerprint.
     */
    private int mNumRangingReadings;

    /**
     * Number of RSSI readings found on provided fingerprint.
     */
    private int mNumRssiReadings;

    /**
     * Constructor.
     */
    public SequentialRobustMixedPositionEstimator() { }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(
            List<? extends RadioSourceLocated<P>> sources) {
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing reagins at an unknown location
     *                      for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public SequentialRobustMixedPositionEstimator(
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sources   located radio sources used for lateration.
     * @param listener  listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public SequentialRobustMixedPositionEstimator(
            List<? extends RadioSourceLocated<P>> sources,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sources);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param fingerprint   fingerprint containing readings at an unknown location for
     *                      provided located radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(fingerprint);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sources       located radio sources used for lateration.
     * @param fingerprint   fingerprint containing readings at an unknown location
     *                      for provided located radio sources.
     * @param listener      listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sources, fingerprint);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to
     *                                          readings within provided fingerprint.
     *                                          The larger the score the better the
     *                                          quality of the reading.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores) {
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param sources                           located radio sources used for
     *                                          lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            List<? extends RadioSourceLocated<P>> sources) {
        this(sources);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param fingerprint                       fingerprint containing readings at an
     *                                          unknown location for provided located
     *                                          radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param sources                           located radio sources used for
     *                                          lateration.
     * @param fingerprint                       fingerprint containing readings at an
     *                                          unknown location for provided located
     *                                          radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        this(sources, fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param listener                          listener in charge of handling events.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param sources                           located radio sources used for
     *                                          lateration.
     * @param listener                          listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            List<? extends RadioSourceLocated<P>> sources,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, sources);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to
     *                                          readings within provided fingerprint.
     *                                          The larger the score the better the
     *                                          quality of the reading.
     * @param fingerprint                       fingerprint containing readings at an
     *                                          unknown location for provided located
     *                                          radio sources.
     * @param listener                          listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, fingerprint);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The
     *                                          larger the score value the better the
     *                                          quality of the radio source.
     * @param fingerprintReadingQualityScores   quality scores corresponding to readings
     *                                          within provided fingerprint. The larger
     *                                          the score the better the quality of the
     *                                          reading.
     * @param sources                           located radio sources used for
     *                                          lateration.
     * @param fingerprint                       fingerprint containing readings at an
     *                                          unknown location for provided located radio
     *                                          sources.
     * @param listener                          listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator(double[] sourceQualityScores,
            double[] fingerprintReadingQualityScores,
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            SequentialRobustMixedPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, sources,
                fingerprint);
        mListener = listener;
    }

    /**
     * Gets robust method used for robust position estimation using ranging data.
     *
     * @return robust method used for robust position estimation using ranging data.
     */
    public RobustEstimatorMethod getRangingRobustMethod() {
        return mRangingRobustMethod;
    }

    /**
     * Sets robust method for robust position estimation using ranging data.
     *
     * @param rangingRobustMethod   robust method used for robust position estimation
     *                              using ranging data.
     * @throws LockedException  if this instance is locked.
     */
    public void setRangingRobustMethod(RobustEstimatorMethod rangingRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mRangingRobustMethod = rangingRobustMethod;
    }

    /**
     * Gets robust method used for coarse robust position estimation using RSSI data.
     *
     * @return robust method used for coarse robust position estimation using RSSI
     * data.
     */
    public RobustEstimatorMethod getRssiRobustMethod() {
        return mRssiRobustMethod;
    }

    /**
     * Sets robust method used for coarse robust position estimation using RSSI data.
     *
     * @param rssiRobustMethod  robust method used for coarse robust position estimation
     *                          using RSSI data.
     * @throws LockedException  if this instance is locked.
     */
    public void setRssiRobustMethod(RobustEstimatorMethod rssiRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mRssiRobustMethod = rssiRobustMethod;
    }

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     * @return true to take into account radio source position covariance during ranging
     * position estimation, false otherwise.
     */
    public boolean isRangingRadioSourcePositionCovarianceUsed() {
        return mUseRangingRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     * @param useRangingRadioSourcePositionCovariance   true to take into account radio
     *                                                  source position covariance during
     *                                                  ranging position estimation,
     *                                                  false otherwise.
     * @throws LockedException  if this instance is locked.
     */
    public void setRangingRadioSourcePositionCovarianceUsed(
            boolean useRangingRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mUseRangingRadioSourcePositionCovariance =
                useRangingRadioSourcePositionCovariance;
    }

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for RSSI measurements.
     *
     * @return true to take into account radio source position covariance during RSSI
     * position estimation, false otherwise.
     */
    public boolean isRssiRadioSourcePositionCovarianceUsed() {
        return mUseRssiRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for RSSI measurements.
     * @param useRssiRadioSourcePositionCovariance  true to take into account radio
     *                                              source position covariance during
     *                                              RSSI position estimation, false
     *                                              otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiRadioSourcePositionCovarianceUsed(
            boolean useRssiRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mUseRssiRadioSourcePositionCovariance = useRssiRadioSourcePositionCovariance;
    }

    /**
     * Indicates whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     *
     * @return true if ranging readings are evenly distributed among radio sources,
     * false otherwise.
     */
    public boolean isRangingReadingsEvenlyDistributed() {
        return mEvenlyDistributeRangingReadings;
    }

    /**
     * Specifies whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     *
     * @param evenlyDistributeRangingReadings   true if ranging readings are evenly
     *                                          distributed among radio sources, false
     *                                          otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingReadingsEvenlyDistributed(
            boolean evenlyDistributeRangingReadings) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mEvenlyDistributeRangingReadings = evenlyDistributeRangingReadings;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     *
     * @return distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     */
    public double getRssiFallbackDistanceStandardDeviation() {
        return mRssiFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     *
     * @param rssiFallbackDistanceStandardDeviation distance standard deviation fallback
     *                                              value to use when none can be
     *                                              determined from provided RSSI
     *                                              measurements.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiFallbackDistanceStandardDeviation(
            double rssiFallbackDistanceStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRssiFallbackDistanceStandardDeviation = rssiFallbackDistanceStandardDeviation;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     *
     * @return distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     */
    public double getRangingFallbackDistanceStandardDeviation() {
        return mRangingFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     *
     * @param rangingFallbackDistanceStandardDeviation  distance standard deviation
     *                                                  fallback value to use when none
     *                                                  can be determined from provided
     *                                                  ranging measurements.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingFallbackDistanceStandardDeviation(
            double rangingFallbackDistanceStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRangingFallbackDistanceStandardDeviation =
                rangingFallbackDistanceStandardDeviation;
    }

    /**
     * Indicates whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     *
     * @return true if RSSI readings are evenly distributed among radio sources,
     * false otherwise.
     */
    public boolean isRssiReadingsEvenlyDistributed() {
        return mEvenlyDistributeRssiReadings;
    }

    /**
     * Specifies whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     *
     * @param evenlyDistributeRssiReadings  true if RSSI readings are evenly distributed
     *                                      among radio sources, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiReadingsEvenlyDistributed(
            boolean evenlyDistributeRssiReadings) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mEvenlyDistributeRssiReadings = evenlyDistributeRssiReadings;
    }

    /**
     * Gets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     * greater than 1.
     * @throws LockedException          if this instance is locked.
     */
    public void setProgressDelta(float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%) for robust position estimation on ranging data. The
     * amount of confidence indicates the probability that the estimated result is
     * correct. Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust position estimation as a value between
     * 0.0 and 1.0.
     */
    public double getRangingConfidence() {
        return mRangingConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on ranging data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @param rangingConfidence confidence to be set for robust position estimation as
     *                          a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRangingConfidence(double rangingConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingConfidence < MIN_CONFIDENCE || rangingConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mRangingConfidence = rangingConfidence;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount of
     * confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust position estimation as a value between
     * 0.0 and 1.0.
     */
    public double getRssiConfidence() {
        return mRssiConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @param rssiConfidence    amount of confidence for robust position estimation as
     *                          a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRssiConfidence(double rssiConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiConfidence < MIN_CONFIDENCE || rssiConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mRssiConfidence = rssiConfidence;
    }

    /**
     * Gets maximum allowed number of iterations for robust ranging position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRangingMaxIterations() {
        return mRangingMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust ranging position
     * estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @param rangingMaxIterations  maximum allowed number of iterations to be set for
     *                              position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if estimator is locked.
     */
    public void setRangingMaxIterations(int rangingMaxIterations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingMaxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mRangingMaxIterations = rangingMaxIterations;
    }

    /**
     * Gets maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRssiMaxIterations() {
        return mRssiMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @param rssiMaxIterations maximum allowed number of iterations to be set for
     *                          position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if estimator is locked.
     */
    public void setRssiMaxIterations(int rssiMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiMaxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mRssiMaxIterations = rssiMaxIterations;
    }

    /**
     * Indicates whether result is refined using all found inliers.
     *
     * @return true if result is refined, false otherwise.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result is refined using all found inliers.
     *
     * @param refineResult true if result is refined, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setResultRefined(boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mRefineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Indicates whether a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @return true if a linear solver is used for preliminary solution estimation on
     * ranging readings.
     */
    public boolean isRangingLinearSolverUsed() {
        return mUseRangingLinearSolver;
    }

    /**
     * Specifies whether a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @param useRangingLinearSolver    true if a linear solver is used for preliminary
     *                                  solution estimation on ranging readings.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingLinearSolverUsed(boolean useRangingLinearSolver)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRangingLinearSolver = useRangingLinearSolver;
    }

    /**
     * Indicates whether a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @return true if a linear solver is used for preliminary solution estimation on
     * RSSI readings.
     */
    public boolean isRssiLinearSolverUsed() {
        return mUseRssiLinearSolver;
    }

    /**
     * Specifies whether a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @param useRssiLinearSolver   true if a linear solver is used for preliminary
     *                              solution estimation on RSSI readings.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiLinearSolverUsed(boolean useRssiLinearSolver)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRssiLinearSolver = useRssiLinearSolver;
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     *
     * @return true to use an homogeneous linear solver for preliminary solutions
     * during ranging fine position estimation.
     */
    public boolean isRangingHomogeneousLinearSolverUsed() {
        return mUseRangingHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     *
     * @param useRangingHomogeneousLinearSolver true to use an homogeneous linear
     *                                          solver for preliminary solutions during
     *                                          ranging fine position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingHomogeneousLinearSolverUsed(
            boolean useRangingHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRangingHomogeneousLinearSolver = useRangingHomogeneousLinearSolver;
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     *
     * @return true to use an homogeneous linear solver for preliminary solutions
     * during RSSI coarse position estimation.
     */
    public boolean isRssiHomogeneousLinearSolverUsed() {
        return mUseRssiHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     *
     * @param useRssiHomogeneousLinearSolver    true to use an homogeneous linear
     *                                          solver for preliminary solutions during
     *                                          RSSI fine position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiHomogeneousLinearSolverUsed(
            boolean useRssiHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRssiHomogeneousLinearSolver = useRssiHomogeneousLinearSolver;
    }

    /**
     * Indicates whether preliminary ranging solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non
     * linear solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @return true if preliminary ranging solutions must be refined after an initial
     * linear solution, false otherwise.
     */
    public boolean isRangingPreliminarySolutionRefined() {
        return mRefineRangingPreliminarySolutions;
    }

    /**
     * Specifies whether preliminary ranging solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non
     * linear solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @param refineRangingPreliminarySolutions true if preliminary ranging solutions
     *                                          must be refined after an initial linear
     *                                          solution, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingPreliminarySolutionRefined(
            boolean refineRangingPreliminarySolutions) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefineRangingPreliminarySolutions = refineRangingPreliminarySolutions;
    }

    /**
     * Indicates whether preliminary RSSI solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non
     * linear solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @return true if preliminary RSSI solutions must be refined after an initial
     * linear solution, false otherwise.
     */
    public boolean isRssiPreliminarySolutionRefined() {
        return mRefineRssiPreliminarySolutions;
    }

    /**
     * Specifies whether preliminary RSSI solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non
     * linear solver will be used regardless of this value using an average solution
     * as the initial value ot be refined.
     *
     * @param refineRssiPreliminarySolutions    true if preliminary RSSI solutions must
     *                                          be refined after an initial linear
     *                                          solution, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiPreliminarySolutionRefined(
            boolean refineRssiPreliminarySolutions) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefineRssiPreliminarySolutions = refineRssiPreliminarySolutions;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * fine ranging position estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for ranging estimation or null.
     */
    public Double getRangingThreshold() {
        return mRangingThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * fine ranging position estimation.
     * If not defined, default threshold will be used.
     *
     * @param rangingThreshold threshold for ranging estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingThreshold(Double rangingThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRangingThreshold = rangingThreshold;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * coarse RSSI position estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for RSSI estimation or null.
     */
    public Double getRssiThreshold() {
        return mRssiThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * coarse RSSI position estimation.
     * If not defined, default threshold will be used.
     *
     * @param rssiThreshold threshold for RSSI estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiThreshold(Double rssiThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRssiThreshold = rssiThreshold;
    }

    /**
     * Gets located radio sources used for lateration.
     *
     * @return located radio sources used for lateration.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Sets located radio sources used for lateration.
     *
     * @param sources   located radio sources used for lateration.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public void setSources(List<? extends RadioSourceLocated<P>> sources)
            throws LockedException{
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located
     * radio sources.
     *
     * @return fingerprint containing readings at an unknown location for provided
     * located radio sources.
     */
    public Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> getFingerprint() {
        return mFingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided
     * located radio sources.
     *
     * @param fingerprint   fingerprint containing readings at an unknown location for
     *                      provided located radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the radio source.
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
    public void setSourceQualityScores(double[] sourceQualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSourceQualityScores(sourceQualityScores);
    }

    /**
     * Gets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     *
     * @return quality scores corresponding to each reading within provided fingerprint.
     */
    public double[] getFingerprintReadingsQualityScores() {
        return mFingerprintReadingsQualityScores;
    }

    /**
     * Sets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     *
     * @param fingerprintReadingsQualityScores  quality scores corresponding to each
     *                                          reading within provided fingerprint.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setFingerprintReadingsQualityScores(
            double[] fingerprintReadingsQualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public SequentialRobustMixedPositionEstimatorListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            SequentialRobustMixedPositionEstimatorListener<P> listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @return an initial position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @param initialPosition   an initial position.
     * @throws LockedException  if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is under
     * progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        checkFingerprint(mFingerprint);

        int numSources = mSources != null ? mSources.size() : 0;
        return numSources > getMinRequiredSources() &&
                (mRssiEstimatorAvailable || mRangingEstimatorAvailable);
    }

    /**
     * Estimates position based on provided located radio sources and readings of such
     * sources at an unknown location.
     *
     * @return estimated position.
     * @throws LockedException          if estimator is locked.
     * @throws NotReadyException        if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for some other reason.
     */
    public P estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        if (mRssiEstimatorAvailable) {
            buildRssiEstimator();
            setupRssiEstimator();

            if (!mRssiEstimator.isReady()) {
                throw new NotReadyException();
            }
        }

        if (mRangingEstimatorAvailable) {
            buildRangingEstimator();
            setupRangingEstimator();

            if (!mRangingEstimator.isReady()) {
                throw new NotReadyException();
            }
        }

        mLocked = true;
        if (mListener != null) {
            mListener.onEstimateStart(this);
        }

        P coarsePosition = mInitialPosition;
        if (mRssiEstimator != null) {
            mRssiEstimator.setInitialPosition(mInitialPosition);
            try {
                // estimate coarse position using RSSI data
                coarsePosition = mRssiEstimator.estimate();
            } catch (RobustEstimatorException e) {
                coarsePosition = null;
            }
        }

        // use coarse position as initial position for ranging estimation
        if (mRangingEstimator != null) {
            mRangingEstimator.setInitialPosition(
                    coarsePosition != null ? coarsePosition : mInitialPosition);
        }

        try {
            P result = mRangingEstimator != null ?
                    mRangingEstimator.estimate() : coarsePosition;

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }

            return result;
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mRangingEstimator != null ? mRangingEstimator.getInliersData() :
                mRssiEstimator != null ? mRssiEstimator.getInliersData() : null;
    }

    /**
     * Gets known positions of radio sources used internally to solve lateration.
     *
     * @return known positions used internally.
     */
    public P[] getPositions() {
        return mRangingEstimator != null ? mRangingEstimator.getPositions() :
                mRssiEstimator != null ? mRssiEstimator.getPositions() : null;
    }

    /**
     * Gets euclidean distances from known located radio sources to the location of
     * provided readings in a fingerprint.
     * Distance values are used internally to solve lateration.
     *
     * @return euclidean distances used internally.
     */
    public double[] getDistances() {
        return mRangingEstimator != null ? mRangingEstimator.getDistances() :
                mRssiEstimator != null ? mRssiEstimator.getDistances() : null;
    }

    /**
     * Gets standard deviation distances from known located radio sources to the
     * location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve lateration.
     *
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return mRangingEstimator != null ?
                mRangingEstimator.getDistanceStandardDeviations() :
                mRssiEstimator != null ?
                        mRssiEstimator.getDistanceStandardDeviations() : null;
    }

    /**
     * Gets estimated covariance of estimated position if available.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return mRangingEstimator != null ? mRangingEstimator.getCovariance() :
                mRssiEstimator != null ? mRssiEstimator.getCovariance() : null;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mRangingEstimator != null ?
                mRangingEstimator.getEstimatedPosition() :
                mRssiEstimator != null ? mRssiEstimator.getEstimatedPosition() : null;
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return number of dimensions of provided points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform
     * lateration.
     */
    public abstract int getMinRequiredSources();

    /**
     * Builds ranging internal estimator.
     */
    protected abstract void buildRangingEstimator();

    /**
     * Builds RSSI internal estimator.
     */
    protected abstract void buildRssiEstimator();

    /**
     * Setups ranging internal estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    private void setupRangingEstimator() throws LockedException {
        if (mFingerprint != null) {
            //builds separated ranging readings
            List<? extends Reading<? extends RadioSource>> readings =
                    mFingerprint.getReadings();

            List<RangingReading<RadioSource>> rangingReadings =
                    new ArrayList<>();

            double[] fingerprintReadingsQualityScores =
                    new double[mNumRangingReadings];
            int i = 0;
            int j = 0;
            for (Reading<? extends RadioSource> reading : readings) {
                if (reading instanceof RangingReading) {
                    // noinspection unchecked
                    rangingReadings.add(
                            (RangingReading<RadioSource>) reading);
                    fingerprintReadingsQualityScores[i] =
                            mFingerprintReadingsQualityScores[j];
                    i++;
                } else if (reading instanceof RangingAndRssiReading) {
                    // noinspection unchecked
                    rangingReadings.add(createRangingReading(
                            (RangingAndRssiReading<RadioSource>) reading));
                    fingerprintReadingsQualityScores[i] =
                            mFingerprintReadingsQualityScores[j];
                    i++;
                }
                j++;
            }

            RangingFingerprint<RadioSource, RangingReading<RadioSource>> rangingFingerprint =
                    new RangingFingerprint<>(rangingReadings);

            // set data and configuration on both internal estimators
            mRangingEstimator.setSources(mSources);
            mRangingEstimator.setFingerprint(rangingFingerprint);
            mRangingEstimator.setRadioSourcePositionCovarianceUsed(
                    mUseRangingRadioSourcePositionCovariance);
            mRangingEstimator.setEvenlyDistributeReadings(mEvenlyDistributeRangingReadings);
            mRangingEstimator.setFallbackDistanceStandardDeviation(
                    mRangingFallbackDistanceStandardDeviation);
            mRangingEstimator.setProgressDelta(2.0f * mProgressDelta);
            mRangingEstimator.setConfidence(mRangingConfidence);
            mRangingEstimator.setMaxIterations(mRangingMaxIterations);
            mRangingEstimator.setCovarianceKept(mKeepCovariance);
            mRangingEstimator.setInitialPosition(mInitialPosition);
            mRangingEstimator.setLinearSolverUsed(mUseRangingLinearSolver);
            mRangingEstimator.setHomogeneousLinearSolverUsed(
                    mUseRangingHomogeneousLinearSolver);
            mRangingEstimator.setPreliminarySolutionRefined(
                    mRefineRangingPreliminarySolutions);
            mRangingEstimator.setSourceQualityScores(mSourceQualityScores);
            mRangingEstimator.setFingerprintReadingsQualityScores(
                    fingerprintReadingsQualityScores);
            mRangingEstimator.setListener(new RobustRangingPositionEstimatorListener<P>() {
                @Override
                public void onEstimateStart(
                        RobustRangingPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateEnd(
                        RobustRangingPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateNextIteration(
                        RobustRangingPositionEstimator<P> estimator, int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        RobustRangingPositionEstimator<P> estimator, float progress) {
                    if (mListener != null) {
                        float p = mRssiEstimatorAvailable ?
                                0.5f + 0.5f * progress : progress;
                        mListener.onEstimateProgressChange(
                                SequentialRobustMixedPositionEstimator.this,
                                p);
                    }
                }
            });
        }
    }

    /**
     * Setups RSSI internal estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    private void setupRssiEstimator() throws LockedException {
        if (mFingerprint != null) {
            //builds separated RSSI readings
            List<? extends Reading<? extends RadioSource>> readings =
                    mFingerprint.getReadings();

            List<RssiReading<RadioSource>> rssiReadings =
                    new ArrayList<>();

            double[] fingerprintReadingsQualityScores =
                    new double[mNumRssiReadings];
            int i = 0;
            int j = 0;
            for (Reading<? extends RadioSource> reading : readings) {
                if (reading instanceof RssiReading) {
                    // noinspection unchecked
                    rssiReadings.add((RssiReading<RadioSource>) reading);
                    fingerprintReadingsQualityScores[i] =
                            mFingerprintReadingsQualityScores[j];
                    i++;
                } else if (reading instanceof RangingAndRssiReading) {
                    // noinspection unchecked
                    rssiReadings.add(createRssiReading(
                            (RangingAndRssiReading<RadioSource>) reading));
                    fingerprintReadingsQualityScores[i] =
                            mFingerprintReadingsQualityScores[j];
                    i++;
                }
                j++;
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> rssiFingerprint =
                    new RssiFingerprint<>(rssiReadings);

            // set data and configuration on both internal estimators
            mRssiEstimator.setSources(mSources);
            mRssiEstimator.setFingerprint(rssiFingerprint);
            mRssiEstimator.setRadioSourcePositionCovarianceUsed(
                    mUseRssiRadioSourcePositionCovariance);
            mRssiEstimator.setEvenlyDistributeReadings(mEvenlyDistributeRssiReadings);
            mRssiEstimator.setFallbackDistanceStandardDeviation(
                    mRssiFallbackDistanceStandardDeviation);
            mRssiEstimator.setProgressDelta(2.0f * mProgressDelta);
            mRssiEstimator.setConfidence(mRssiConfidence);
            mRssiEstimator.setMaxIterations(mRssiMaxIterations);
            mRssiEstimator.setResultRefined(mRefineResult);
            mRssiEstimator.setCovarianceKept(mKeepCovariance);
            mRssiEstimator.setInitialPosition(mInitialPosition);
            mRssiEstimator.setLinearSolverUsed(mUseRssiLinearSolver);
            mRssiEstimator.setHomogeneousLinearSolverUsed(
                    mUseRssiHomogeneousLinearSolver);
            mRssiEstimator.setPreliminarySolutionRefined(
                    mRefineRssiPreliminarySolutions);
            mRssiEstimator.setSourceQualityScores(mSourceQualityScores);
            mRssiEstimator.setFingerprintReadingsQualityScores(
                    fingerprintReadingsQualityScores);
            mRssiEstimator.setListener(new RobustRssiPositionEstimatorListener<P>() {
                @Override
                public void onEstimateStart(
                        RobustRssiPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateEnd(
                        RobustRssiPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateNextIteration(
                        RobustRssiPositionEstimator<P> estimator, int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        RobustRssiPositionEstimator<P> estimator, float progress) {
                    if (mListener != null) {
                        float p = mRangingEstimatorAvailable ?
                                0.5f * progress : progress;
                        mListener.onEstimateProgressChange(
                                SequentialRobustMixedPositionEstimator.this,
                                p);
                    }
                }
            });
        }
    }

    /**
     * Internally sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     * provided sources is less than the required minimum.
     */
    private void internalSetSources(List<? extends RadioSourceLocated<P>> sources) {
        if (sources == null) {
            throw new IllegalArgumentException();
        }

        if (sources.size() < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mSources = sources;
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for
     * provided located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetFingerprint(Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        mFingerprint = fingerprint;
    }

    /**
     * Sets quality scores corresponding to each provided located radio source.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param sourceQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than 3 samples for 2D or 4 samples for 3D.
     */
    private void internalSetSourceQualityScores(double[] sourceQualityScores) {
        if (sourceQualityScores == null ||
                sourceQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mSourceQualityScores = sourceQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided reading within provided
     * fingerprint.
     * This method is used internally and does not check whether instance is locked
     * or not.
     * @param fingerprintReadingsQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     * than 3 samples for 2D or 4 samples for 3D.
     */
    private void internalSetFingerprintReadingsQualityScores(
            double[] fingerprintReadingsQualityScores) {
        if (fingerprintReadingsQualityScores == null ||
                fingerprintReadingsQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        mFingerprintReadingsQualityScores = fingerprintReadingsQualityScores;
    }

    /**
     * Creates a ranging reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return a ranging reading containing only the ranging data of input reading.
     */
    private RangingReading<RadioSource> createRangingReading(
            RangingAndRssiReading<? extends RadioSource> reading) {
        return new RangingReading<>(reading.getSource(),
                reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                reading.getNumAttemptedMeasurements(),
                reading.getNumSuccessfulMeasurements());
    }

    /**
     * Creates an RSSI reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return an RSSI reading containing only the RSSI data of input reading.
     */
    private RssiReading<RadioSource> createRssiReading(
            RangingAndRssiReading<? extends RadioSource> reading) {
        return new RssiReading<>(reading.getSource(), reading.getRssi(),
                reading.getRssiStandardDeviation());
    }

    /**
     * Checks readings within provided fingerprint to determine the amount of available
     * ranging or RSSI readings.
     *
     * @param fingerprint fingerprint to be checked.
     */
    private void checkFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        checkReadings(fingerprint != null ? fingerprint.getReadings() : null);
    }

    /**
     * Checks number of available ranging readings and number of available RSSI
     * readings. Also determines whether position must be estimated using ranging
     * data or RSSI data.
     * @param readings readings to be checked.
     */
    private void checkReadings(List<? extends Reading> readings) {
        mNumRssiReadings = 0;
        mNumRangingReadings = 0;
        mRangingEstimatorAvailable = mRssiEstimatorAvailable = false;

        if (readings == null) {
            return;
        }

        for (Reading reading : readings) {
            if (reading instanceof RangingReading) {
                mNumRangingReadings++;
            } else if (reading instanceof RssiReading) {
                mNumRssiReadings++;
            } else if (reading instanceof RangingAndRssiReading) {
                mNumRangingReadings++;
                mNumRssiReadings++;
            }
        }

        int min = getMinRequiredSources();
        mRangingEstimatorAvailable = mNumRangingReadings >= min;
        mRssiEstimatorAvailable = mNumRssiReadings >= min;
    }
}
