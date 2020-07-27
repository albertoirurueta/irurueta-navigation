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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.algebra.AlgebraException;
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
 * This is an abstract class to robustly estimate position, transmitted power and pathloss
 * exponent of a radio source (e.g. WiFi access point or bluetooth beacon), by discarding
 * outliers and assuming that the ranging data is available to obtain position with
 * greater accuracy and that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * <p>
 * Implementations of this class sequentially estimate position and then remaining
 * parameters. First ranging data is used to robustly estimate position and then
 * remaining parameters are robustly estimated using former estimated position as
 * an initial guess.
 * <p>
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * <p>
 * Implementations of this class might produce more stable positions of estimated
 * radio sources than implementations of RobustRangingAndRssiRadioSourceEstimator.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class SequentialRobustRangingAndRssiRadioSourceEstimator<S extends RadioSource,
        P extends Point<P>> {

    /**
     * Default robust estimator method for robust position estimation using ranging
     * data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_PANGING_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Default robust estimator method for pathloss exponent and transmitted power
     * estimation using RSSI data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_RSSI_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that result is refined by default using all found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

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
     * Indicates that by default position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    public static final boolean DEFAULT_USE_READING_POSITION_COVARIANCES = true;

    /**
     * Internal robust estimator for position estimation.
     */
    protected RobustRangingRadioSourceEstimator<S, P> mRangingEstimator;

    /**
     * Internal robust estimator for pathloss exponent and transmitted power
     * estimation.
     */
    protected RobustRssiRadioSourceEstimator<S, P> mRssiEstimator;

    /**
     * Robust method used for robust position estimation using ranging data.
     */
    protected RobustEstimatorMethod mRangingRobustMethod = DEFAULT_PANGING_ROBUST_METHOD;

    /**
     * Robust method used for pathloss exponent and transmitted power estimation
     * using RSSI data.
     */
    protected RobustEstimatorMethod mRssiRobustMethod = DEFAULT_RSSI_ROBUST_METHOD;

    /**
     * Size of subsets to be checked during ranging robust estimation.
     */
    protected int mRangingPreliminarySubsetSize;

    /**
     * Size of subsets to be checked during RSSI robust estimation.
     */
    protected int mRssiPreliminarySubsetSize;

    /**
     * Threshold to determine when samples are inliers or not used during robust
     * position estimation.
     * If not defined, default threshold will be used.
     */
    protected Double mRangingThreshold;

    /**
     * Threshold to determine when samples are inliers or not used during robust
     * pathloss exponent and transmitted power estimation.
     */
    protected Double mRssiThreshold;

    /**
     * Signal readings belonging to the same radio source to be estimated.
     */
    private List<? extends RangingAndRssiReadingLocated<S, P>> mReadings;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Listener to be notified of events such as when estimation starts, ends or its
     * progress significantly changes.
     */
    private SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> mListener;

    /**
     * Estimated position.
     */
    private P mEstimatedPosition;

    /**
     * Indicates if this instance is locked because estimation is being executed.
     */
    private boolean mLocked;

    /**
     * Amount of progress variation before notifying a progress change during estimation.
     */
    private float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%) for robust position estimation. The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will be
     * close to 1.0, but not exactly 1.0.
     */
    private double mRangingConfidence = DEFAULT_CONFIDENCE;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%) for robust pathloss exponent and transmitted power estimation. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     */
    private double mRssiConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations for robust position estimation. When the
     * maximum number of iterations is exceeded, an approximate result might be
     * available for retrieval.
     */
    private int mRangingMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Maximum allowed number of iteratios for robust pathloss exponent and transmitted
     * power estimation. When the maximum number of iterations is exceeded, an
     * approximate result might be available for retrieval.
     */
    private int mRssiMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Indicates whether result must be refined using found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    private boolean mRefineResult = DEFAULT_REFINE_RESULT;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Covariance of estimated position, power and/or pathloss exponent.
     * This is only available when result has been refined and covariance is kept.
     */
    private Matrix mCovariance;

    /**
     * Covariance of estimated position.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This value will only be available when position estimation is enabled.
     */
    private Matrix mEstimatedPositionCovariance;

    /**
     * Initially transmitted power to start the estimation of radio source
     * transmitted power.
     * If not defined, average value of received power readings will be used.
     */
    private Double mInitialTransmittedPowerdBm;

    /**
     * Initial position to start the estimation of radio source position.
     * If not defined, centroid of provided located readings will be used.
     */
    private P mInitialPosition;

    /**
     * Initial exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     */
    private double mInitialPathLossExponent =
            RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     */
    private boolean mTransmittedPowerEstimationEnabled =
            RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED;

    /**
     * Indicates whether path loss estimation is enabled or not.
     */
    private boolean mPathLossEstimationEnabled =
            RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED;

    /**
     * Estimated transmitted power expressed in dBm's.
     */
    private double mEstimatedTransmittedPowerdBm;

    /**
     * Estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link RssiRadioSourceEstimator#DEFAULT_PATH_LOSS_EXPONENT}
     */
    private double mEstimatedPathLossExponent =
            RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Variance of estimated transmitted power.
     * This value will only be available when transmitted power
     * estimation is enabled.
     */
    private Double mEstimatedTransmittedPowerVariance;

    /**
     * Variance of estimated path loss exponent.
     * This value will only be available when pathloss
     * exponent estimation is enabled.
     */
    private Double mEstimatedPathLossExponentVariance;

    /**
     * Data related to inliers found after estimation.
     */
    private InliersData mInliersData;

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    private boolean mUseReadingPositionCovariances = DEFAULT_USE_READING_POSITION_COVARIANCES;

    /**
     * Indicates whether an homogeneous ranging linear solver is used to estimate preliminary positions.
     */
    private boolean mUseHomogeneousRangingLinearSolver =
            RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Constructor.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator() {
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings) {
        internalSetReadings(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings);
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition) {
        this(readings);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition) {
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final Double initialTransmittedPowerdBm) {
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm) {
        this(readings);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        this(readings);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        this(initialPosition, initialTransmittedPowerdBm);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructors.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @throws IllegalArgumentException if quality scores is null, or length of
     *                                  quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores) {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings) {
        this(readings);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition) {
        this(readings, initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition) {
        this(initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm) {
        this(initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm) {
        this(readings, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        this(initialPosition, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        this(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        this(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructors.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether estimator is locked during estimation.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
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
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if this estimator is locked.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Gets robust method used for robust position estimation using ranging data.
     *
     * @return robust method used for robust position estimation.
     */
    public RobustEstimatorMethod getRangingRobustMethod() {
        return mRangingRobustMethod;
    }

    /**
     * Sets robust method used for robust position estimation using ranging data.
     *
     * @param rangingRobustMethod robust method used for robust position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingRobustMethod(final RobustEstimatorMethod rangingRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRangingRobustMethod = rangingRobustMethod;
    }

    /**
     * Gets robust method used for pathloss exponent and transmitted power estimation
     * using RSSI data.
     *
     * @return robust method used for pathloss exponent and transmitted power
     * estimation.
     */
    public RobustEstimatorMethod getRssiRobustMethod() {
        return mRssiRobustMethod;
    }

    /**
     * Sets robust method used for pathloss exponent and transmitted power estimation
     * using RSSI data.
     *
     * @param rssiRobustMethod robust method used for pathloss exponent and transmitted
     *                         power estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiRobustMethod(final RobustEstimatorMethod rssiRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRssiRobustMethod = rssiRobustMethod;
    }

    /**
     * Gets size of subsets to be checked during ranging robust estimation.
     *
     * @return size of subsets to be checked during ranging robust estimation.
     */
    public int getRangingPreliminarySubsetSize() {
        return mRangingPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during ranging robust estimation.
     *
     * @param rangingPreliminarySubsetSize size of subsets to be checked during
     *                                     ranging robust estimation.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinReadings()}.
     */
    public void setRangingPreliminarySubsetSize(final int rangingPreliminarySubsetSize)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingPreliminarySubsetSize < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mRangingPreliminarySubsetSize = rangingPreliminarySubsetSize;
    }

    /**
     * Gets size of subsets to be checked during RSSI robust estimation.
     *
     * @return size of subsets to be checked during RSSI robust estimation.
     */
    public int getRssiPreliminarySubsetSize() {
        return mRssiPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during RSSI robust estimation.
     *
     * @param rssiPreliminarySubsetSize size of subsets to be checked during
     *                                  RSSI robust estimation.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinReadings()}.
     */
    public void setRssiPreliminarySubsetSize(final int rssiPreliminarySubsetSize)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiPreliminarySubsetSize < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mRssiPreliminarySubsetSize = rssiPreliminarySubsetSize;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * position estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for ranging estimation or null.
     */
    public Double getRangingThreshold() {
        return mRangingThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * position estimation.
     * If not defined, default threshold will be used.
     *
     * @param rangingThreshold threshold for ranging estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingThreshold(final Double rangingThreshold)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRangingThreshold = rangingThreshold;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * pathloss exponent and transmitted power estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for RSSI estimation or null.
     */
    public Double getRssiThreshold() {
        return mRssiThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * pathloss exponent and transmitted power estimation.
     * If not defined, default threshold will be used.
     *
     * @param rssiThreshold threshold for RSSI estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiThreshold(final Double rssiThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRssiThreshold = rssiThreshold;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%) for robust position estimation. The amount of
     * confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust position estimation as a value
     * between 0.0 and 1.0.
     */
    public double getRangingConfidence() {
        return mRangingConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation. The amount of confidence
     * indicates the probability that the estimated result is correct. Usually this
     * value will be close to 1.0, but not exactly 1.0.
     *
     * @param rangingConfidence confidence to be set for robust position estimation
     *                          as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRangingConfidence(final double rangingConfidence)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingConfidence < MIN_CONFIDENCE || rangingConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mRangingConfidence = rangingConfidence;
    }

    /**
     * Returns amoung of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%) for pathloss exponent and transmitted power
     * estimation. The amount of confidence indicates the probability that the
     * estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust pathloss exponent and transmitted power
     * estimation as a value between 0.0 and 1.0.
     */
    public double getRssiConfidence() {
        return mRssiConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%) for pathloss exponent and transmitted power
     * estimation. The amount of confidence indicates the probability that the
     * estimated result is correct. Usually this value will be close to 10.0, but
     * not exactly 1.0.
     *
     * @param rssiConfidence confidence to be set for robust pathloss exponent and
     *                       transmitted power estimation as a value between 0.0 and
     *                       1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRssiConfidence(final double rssiConfidence)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiConfidence < MIN_CONFIDENCE || rssiConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mRssiConfidence = rssiConfidence;
    }

    /**
     * Returns maximum allowed number of iterations for robust position estimation. If
     * maximum allowed number of iterations is achieved without converging to a result
     * when calling estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRangingMaxIterations() {
        return mRangingMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust position estimation. When
     * the maximum number of iterations is exceeded, an approximate result might be
     * available for retrieval.
     *
     * @param rangingMaxIterations maximum allowed number of iterations to be set
     *                             for position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked.
     */
    public void setRangingMaxIterations(final int rangingMaxIterations)
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
     * Returns maximum allowed number of iterations for robust pathloss exponent and
     * transmitted power estimation. If maximum allowed number of iterations is achieved
     * without converging to a result when calling estimate(), a RobustEstimatorException
     * will be raised.
     *
     * @return maximum allowed number of iterations for pathloss exponent and transmitted
     * power estimation.
     */
    public int getRssiMaxIterations() {
        return mRssiMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust pathloss exponent and
     * transmitted power estimation. When the maximum number of iterations is exceeded,
     * an approximate result might be available for retrieval.
     *
     * @param rssiMaxIterations maximum allowed number of iterations to be set for
     *                          pathloss exponent and transmitted power estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked.
     */
    public void setRssiMaxIterations(final int rssiMaxIterations)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiMaxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mRssiMaxIterations = rssiMaxIterations;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
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
    public void setCovarianceKept(final boolean keepCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Gets signal readings belonging to the same radio source.
     *
     * @return signal readings belonging to the same radio source.
     */
    public List<? extends RangingAndRssiReadingLocated<S, P>> getReadings() {
        return mReadings;
    }

    /**
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same
     *                 radio source.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public void setReadings(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetReadings(readings);
    }

    /**
     * Gets listener in charge of attending events raised by this instance.
     *
     * @return listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> getListener() {
        return mListener;
    }

    /**
     * Sets listener in charge of attending events raised by this instance.
     *
     * @param listener listener in charge of attending events raised by this
     *                 instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, P> listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each sample.
     */
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of positions and
     * distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if robust solver is locked because an
     *                                  estimation is already in progress.
     */
    public void setQualityScores(final double[] qualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     *
     * @return initial transmitted power to start the estimation of radio source
     * transmitted power.
     */
    public Double getInitialTransmittedPowerdBm() {
        return mInitialTransmittedPowerdBm;
    }

    /**
     * Sets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted
     *                                   power.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialTransmittedPowerdBm(
            final Double initialTransmittedPowerdBm) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     *
     * @return initial transmitted power to start the estimation of radio source
     * transmitted power.
     */
    public Double getInitialTransmittedPower() {
        return mInitialTransmittedPowerdBm != null ?
                Utils.dBmToPower(mInitialTransmittedPowerdBm) : null;
    }

    /**
     * Sets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     *
     * @param initialTransmittedPower initial transmitted power to start the
     *                                estimation of radio source transmitted power.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setInitialTransmittedPower(final Double initialTransmittedPower)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (initialTransmittedPower != null) {
            if (initialTransmittedPower < 0.0) {
                throw new IllegalArgumentException();
            }
            mInitialTransmittedPowerdBm = Utils.powerTodBm(
                    initialTransmittedPower);
        } else {
            mInitialTransmittedPowerdBm = null;
        }
    }

    /**
     * Gets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     *
     * @return initial position to start the estimation of radio source position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Gets initial exponent typically used on free space for path loss propagation
     * in terms of distance.
     * On different environments path loss exponent might have different value:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     *
     * @return initial path loss exponent.
     */
    public double getInitialPathLossExponent() {
        return mInitialPathLossExponent;
    }

    /**
     * Sets initial exponent typically used on free space for path loss propagation
     * in terms of distance.
     * On different environments path loss exponent might have different value:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     *
     * @param initialPathLossExponent initial path loss exponent.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPathLossExponent(final double initialPathLossExponent)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     *
     * @return true if transmitted power estimation is enabled, false otherwise.
     */
    public boolean isTransmittedPowerEstimationEnabled() {
        return mTransmittedPowerEstimationEnabled;
    }

    /**
     * Specifies whether transmitted power estimation is enabled or not.
     *
     * @param transmittedPowerEstimationEnabled true if transmitted power estimation is enabled,
     *                                          false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setTransmittedPowerEstimationEnabled(
            final boolean transmittedPowerEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mTransmittedPowerEstimationEnabled = transmittedPowerEstimationEnabled;
    }

    /**
     * Indicates whether path loss estimation is enabled or not.
     *
     * @return true if path loss estimation is enabled, false otherwise.
     */
    public boolean isPathLossEstimationEnabled() {
        return mPathLossEstimationEnabled;
    }

    /**
     * Specifies whether path loss estimation is enabled or not.
     *
     * @param pathLossEstimationEnabled true if path loss estimation is enabled,
     *                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPathLossEstimationEnabled(
            final boolean pathLossEstimationEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPathLossEstimationEnabled = pathLossEstimationEnabled;
    }

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     *
     * @return true to take into account reading position covariances, false otherwise.
     */
    public boolean getUseReadingPositionCovariance() {
        return mUseReadingPositionCovariances;
    }

    /**
     * Specifies whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     *
     * @param useReadingPositionCovariances true to take into account reading position covariances, false
     *                                      otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseReadingPositionCovariances(
            final boolean useReadingPositionCovariances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseReadingPositionCovariances = useReadingPositionCovariances;
    }

    /**
     * Indicates whether an homogeneous ranging linear solver is used to estimate preliminary
     * positions.
     *
     * @return true if homogeneous ranging linear solver is used, false if an inhomogeneous ranging linear
     * one is used instead.
     */
    public boolean isHomogeneousRangingLinearSolverUsed() {
        return mUseHomogeneousRangingLinearSolver;
    }

    /**
     * Specifies whether an homogeneous ranging linear solver is used to estimate preliminary
     * positions.
     *
     * @param useHomogeneousRangingLinearSolver true if homogeneous ranging linear solver is used, false
     *                                          if an inhomogeneous ranging linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    public void setHomogeneousRangingLinearSolverUsed(
            final boolean useHomogeneousRangingLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mUseHomogeneousRangingLinearSolver = useHomogeneousRangingLinearSolver;
    }

    /**
     * Gets covariance for estimated position and power.
     * Matrix contains information in the following order:
     * Top-left submatrix contains covariance of position,
     * then follows transmitted power variance, and finally
     * the last element contains pathloss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return covariance for estimated position and power.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated position covariance.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated position covariance.
     */
    public Matrix getEstimatedPositionCovariance() {
        return mEstimatedPositionCovariance;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mEstimatedPosition;
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     *
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    public boolean areValidReadings(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings) {
        return readings != null && readings.size() >= getMinReadings();
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     * @throws LockedException if estimator is locked
     */
    public boolean isReady() throws LockedException {
        buildRangingEstimatorIfNeeded();
        setupRangingEstimator();

        if (mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled) {
            buildRssiEstimatorIfNeeded();
            setupRssiEstimator();
        }

        return mRangingEstimator.isReady() &&
                ((!mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled) ||
                        mRssiEstimator.isReady());
    }

    /**
     * Gets estimated transmitted power variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated transmitted power variance.
     */
    public Double getEstimatedTransmittedPowerVariance() {
        return mEstimatedTransmittedPowerVariance;
    }

    /**
     * Gets estimated path loss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated path loss exponent variance.
     */
    public Double getEstimatedPathLossExponentVariance() {
        return mEstimatedPathLossExponentVariance;
    }

    /**
     * Gets estimated transmitted power expressed in milli watts (mW).
     *
     * @return estimated transmitted power expressed in milli watts.
     */
    public double getEstimatedTransmittedPower() {
        return Utils.dBmToPower(mEstimatedTransmittedPowerdBm);
    }

    /**
     * Gets estimated transmitted power expressed in dBm's.
     *
     * @return estimated transmitted power expressed in dBm's.
     */
    public double getEstimatedTransmittedPowerdBm() {
        return mEstimatedTransmittedPowerdBm;
    }

    /**
     * Gets estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link RssiRadioSourceEstimator#DEFAULT_PATH_LOSS_EXPONENT}
     *
     * @return estimated path loss exponent.
     */
    public double getEstimatedPathLossExponent() {
        return mEstimatedPathLossExponent;
    }

    /**
     * Robustly estimates position, transmitted power and pathloss exponent for a
     * radio source.
     *
     * @throws LockedException          if instance is busy during estimation.
     * @throws NotReadyException        if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public void estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        try {
            mLocked = true;

            //when checking for readiness, inner estimators are created and setup
            if (!isReady()) {
                throw new NotReadyException();
            }


            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            //estimate position
            mRangingEstimator.setPreliminarySubsetSize(mRangingPreliminarySubsetSize);

            mRangingEstimator.estimate();

            mEstimatedPosition = mRangingEstimator.getEstimatedPosition();
            mEstimatedPositionCovariance =
                    mRangingEstimator.getEstimatedPositionCovariance();
            mInliersData = mRangingEstimator.getInliersData();

            //estimate transmitted power and/or pathloss if enabled
            if (mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled) {
                mRssiEstimator.setPositionEstimationEnabled(false);
                mRssiEstimator.setInitialPosition(mEstimatedPosition);
                mRssiEstimator.setPreliminarySubsetSize(mRssiPreliminarySubsetSize);

                mRssiEstimator.estimate();

                if (mTransmittedPowerEstimationEnabled) {
                    //transmitted power estimation enabled
                    mEstimatedTransmittedPowerdBm =
                            mRssiEstimator.getEstimatedTransmittedPowerdBm();
                    mEstimatedTransmittedPowerVariance =
                            mRssiEstimator.getEstimatedTransmittedPowerVariance();
                } else {
                    //transmitted power estimation disabled
                    if (mInitialTransmittedPowerdBm != null) {
                        mEstimatedTransmittedPowerdBm = mInitialTransmittedPowerdBm;
                    }
                    mEstimatedTransmittedPowerVariance = null;
                }

                if (mPathLossEstimationEnabled) {
                    //pathloss exponent estimation enabled
                    mEstimatedPathLossExponent =
                            mRssiEstimator.getEstimatedPathLossExponent();
                    mEstimatedPathLossExponentVariance =
                            mRssiEstimator.getEstimatedPathLossExponentVariance();
                } else {
                    //pathloss exponent estimation disabled
                    mEstimatedPathLossExponent = mInitialPathLossExponent;
                    mEstimatedPathLossExponentVariance = null;
                }

                //build covariance matrix
                final Matrix rssiCov = mRssiEstimator.getCovariance();
                if (mEstimatedPositionCovariance != null && rssiCov != null) {
                    final int dims = getNumberOfDimensions();
                    int n = dims;
                    if (mTransmittedPowerEstimationEnabled) {
                        n++;
                    }
                    if (mPathLossEstimationEnabled) {
                        n++;
                    }

                    final int dimsMinus1 = dims - 1;
                    final int nMinus1 = n - 1;
                    mCovariance = new Matrix(n, n);
                    mCovariance.setSubmatrix(0, 0,
                            dimsMinus1, dimsMinus1,
                            mEstimatedPositionCovariance);
                    mCovariance.setSubmatrix(dims, dims,
                            nMinus1, nMinus1, rssiCov);
                } else {
                    mCovariance = null;
                }
            } else {
                mCovariance = mEstimatedPositionCovariance;
                if (mInitialTransmittedPowerdBm != null) {
                    mEstimatedTransmittedPowerdBm = mInitialTransmittedPowerdBm;
                }
                mEstimatedTransmittedPowerVariance = null;

                mEstimatedPathLossExponent = mInitialPathLossExponent;
                mEstimatedPathLossExponentVariance = null;
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } catch (final AlgebraException e) {
            throw new RobustEstimatorException(e);
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
        return mInliersData;
    }

    /**
     * Gets minimum required number of readings to estimate
     * power, position and pathloss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings for 2D, and 4 readings for 3D.
     *
     * @return minimum required number of readings.
     */
    public abstract int getMinReadings();

    /**
     * Gets number of dimensions of position points.
     *
     * @return number of dimensions of position points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Gets estimated located radio source.
     *
     * @param <S2> type of located radio source.
     * @return estimated located radio source.
     */
    public abstract <S2 extends RadioSourceLocated<P>> S2 getEstimatedRadioSource();

    /**
     * Builds ranging estimator.
     */
    protected abstract void buildRangingEstimatorIfNeeded();

    /**
     * Build RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    protected abstract void buildRssiEstimatorIfNeeded() throws LockedException;

    /**
     * Setups ranging estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    protected void setupRangingEstimator() throws LockedException {
        if (mReadings != null) {
            //build ranging readings
            final List<RangingReadingLocated<S, P>> rangingReadings = new ArrayList<>();
            for (final RangingAndRssiReadingLocated<S, P> reading : mReadings) {
                rangingReadings.add(createRangingReading(reading));
            }
            mRangingEstimator.setReadings(rangingReadings);
        }

        if (mQualityScores != null) {
            mRangingEstimator.setQualityScores(mQualityScores);
        }

        mRangingEstimator.setProgressDelta(2.0f * mProgressDelta);
        mRangingEstimator.setConfidence(mRangingConfidence);
        mRangingEstimator.setMaxIterations(mRangingMaxIterations);
        mRangingEstimator.setResultRefined(mRefineResult);
        mRangingEstimator.setCovarianceKept(mKeepCovariance);
        mRangingEstimator.setUseReadingPositionCovariances(
                mUseReadingPositionCovariances);
        mRangingEstimator.setHomogeneousLinearSolverUsed(
                mUseHomogeneousRangingLinearSolver);

        mRangingEstimator.setInitialPosition(mInitialPosition);

        mRangingEstimator.setListener(new RobustRangingRadioSourceEstimatorListener<S, P>() {
            @Override
            public void onEstimateStart(
                    final RobustRangingRadioSourceEstimator<S, P> estimator) {
                //not used
            }

            @Override
            public void onEstimateEnd(
                    final RobustRangingRadioSourceEstimator<S, P> estimator) {
                //not used
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustRangingRadioSourceEstimator<S, P> estimator,
                    final int iteration) {
                //not used
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustRangingRadioSourceEstimator<S, P> estimator,
                    final float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            SequentialRobustRangingAndRssiRadioSourceEstimator.this,
                            0.5f * progress);
                }
            }
        });
    }

    /**
     * Setups RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    protected void setupRssiEstimator() throws LockedException {
        if (mReadings != null) {
            //build RSSI readings
            final List<RssiReadingLocated<S, P>> rssiReadings = new ArrayList<>();
            for (final RangingAndRssiReadingLocated<S, P> reading : mReadings) {
                rssiReadings.add(createRssiReading(reading));
            }
            mRssiEstimator.setReadings(rssiReadings);
        }

        if (mQualityScores != null) {
            mRssiEstimator.setQualityScores(mQualityScores);
        }

        mRssiEstimator.setProgressDelta(2.0f * mProgressDelta);
        mRssiEstimator.setConfidence(mRssiConfidence);
        mRssiEstimator.setMaxIterations(mRssiMaxIterations);
        mRssiEstimator.setResultRefined(mRefineResult);
        mRssiEstimator.setCovarianceKept(mKeepCovariance);

        //initial position is not set because position estimated from ranging measures
        //will be later used
        mRssiEstimator.setInitialTransmittedPowerdBm(mInitialTransmittedPowerdBm);
        mRssiEstimator.setInitialPathLossExponent(mInitialPathLossExponent);

        mRssiEstimator.setTransmittedPowerEstimationEnabled(
                mTransmittedPowerEstimationEnabled);
        mRssiEstimator.setPathLossEstimationEnabled(mPathLossEstimationEnabled);

        mRssiEstimator.setListener(new RobustRssiRadioSourceEstimatorListener<S, P>() {
            @Override
            public void onEstimateStart(
                    final RobustRssiRadioSourceEstimator<S, P> estimator) {
                //not used
            }

            @Override
            public void onEstimateEnd(
                    final RobustRssiRadioSourceEstimator<S, P> estimator) {
                //not used
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustRssiRadioSourceEstimator<S, P> estimator,
                    final int iteration) {
                //not used
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustRssiRadioSourceEstimator<S, P> estimator,
                    final float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            SequentialRobustRangingAndRssiRadioSourceEstimator.this,
                            0.5f + 0.5f * progress);
                }
            }
        });
    }

    /**
     * Internally sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are null, not enough readings
     *                                  are available, or readings do not belong to the same access point.
     */
    private void internalSetReadings(
            final List<? extends RangingAndRssiReadingLocated<S, P>> readings) {
        if (!areValidReadings(readings)) {
            throw new IllegalArgumentException();
        }

        mReadings = readings;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than required minimum.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null ||
                qualityScores.length < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }

    /**
     * Creates a ranging reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return a ranging reading containing only the ranging data of input reading.
     */
    private RangingReadingLocated<S, P> createRangingReading(
            final RangingAndRssiReadingLocated<S, P> reading) {
        return new RangingReadingLocated<>(reading.getSource(),
                reading.getDistance(), reading.getPosition(),
                reading.getDistanceStandardDeviation(),
                reading.getPositionCovariance());
    }

    /**
     * Creates an RSSI reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return an RSSI reading containing only the RSSI data of input reading.
     */
    private RssiReadingLocated<S, P> createRssiReading(
            final RangingAndRssiReadingLocated<S, P> reading) {
        return new RssiReadingLocated<>(reading.getSource(),
                reading.getRssi(), reading.getPosition(),
                reading.getRssiStandardDeviation(),
                reading.getPositionCovariance());
    }
}
