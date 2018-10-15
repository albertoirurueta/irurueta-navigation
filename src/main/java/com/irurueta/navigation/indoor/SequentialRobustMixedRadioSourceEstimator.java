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
package com.irurueta.navigation.indoor;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
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
 * Gt is the Gain of the transmission antena
 * Gr is the Gain of the receiver antena
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 *
 * Implementations of this class sequentially estimate position and then remaining
 * parameters. First ranging data is used to robustly estimate position and then
 * remaining parameters are robustly estimated using former estimated position as
 * an initial guess.
 *
 * Because usually information about the antena of the radio source cannot be
 * retrieved (because many measurements are made on unkown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 *
 * This implementation is like SequentialRobustRangingAndRssiRadioSourceEstimator but
 * allows mixing different kinds of located radio source readings (ranging, RSSI
 * and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class SequentialRobustMixedRadioSourceEstimator<S extends RadioSource,
        P extends Point> {

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
     * Indicates whether position is estimated using RSSI data.
     * If enough ranging readings are available, this is false and position is estimated using ranging readings,
     * otherwise this is true and position is estimated using RSSI data in a less reliable way.
     */
    protected boolean mRssiPositionEnabled;

    /**
     * Signal readings belonging to the same radio source to be estimated.
     */
    private List<? extends ReadingLocated<P>> mReadings;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Listener to be notified of events such as when estimation starts, ends or its
     * progress significantly changes.
     */
    private SequentialRobustMixedRadioSourceEstimatorListener<S, P> mListener;

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
     * Amoung of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
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
     *
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     */
    private double mInitialPathLossExponent =
            MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     */
    private boolean mTransmittedPowerEstimationEnabled =
            MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED;

    /**
     * Indicates whether path loss estimation is enabled or not.
     */
    private boolean mPathLossEstimationEnabled =
            MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED;

    /**
     * Estimated transmitted power expressed in dBm's or null if not available.
     */
    private Double mEstimatedTransmittedPowerdBm;

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
            MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

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
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    private boolean mUseReadingPositionCovariances = DEFAULT_USE_READING_POSITION_COVARIANCES;

    /**
     * Number of ranging readings available among all readings.
     */
    private int mNumRangingReadings;

    /**
     * Number of RSSI readings available among all readings.
     */
    private int mNumRssiReadings;

    /**
     * Constructor.
     */
    public SequentialRobustMixedRadioSourceEstimator() { }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings)
            throws IllegalArgumentException {
        internalSetReadings(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings);
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition) throws IllegalArgumentException {
        this(readings);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition) {
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustMixedRadioSourceEstimator(
            Double initialTransmittedPowerdBm) {
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        this(readings);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        this(readings);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition,
            Double initialTransmittedPowerdBm) {
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener in charge of attenging events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition,
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener) {
        this(listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent) {
        this(initialPosition, initialTransmittedPowerdBm);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator(P initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructors.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @throws IllegalArgumentException if quality scores is null, or length of
     * quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores)
            throws IllegalArgumentException {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     * null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings)
            throws IllegalArgumentException {
        this(readings);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     * null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     * null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition) throws IllegalArgumentException {
        this(readings, initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            P initialPosition) throws IllegalArgumentException {
        this(initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores, P initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        this(initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        this(readings, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(
            double[] qualityScores, P initialPosition,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        this(initialPosition, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener in charge of attenging events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            P initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        this(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     * of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructors.
     * Sets signal readings belonging to the same radio source.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                     the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     * is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustMixedRadioSourceEstimator(double[] qualityScores,
            List<? extends ReadingLocated<P>> readings,
            P initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
            throws IllegalArgumentException {
        this(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether estimator is locked during estimation.
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * estimation.
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setProgressDelta(float progressDelta)
            throws IllegalArgumentException, LockedException {
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
     * @return robust method used for robust position estimation.
     */
    public RobustEstimatorMethod getRangingRobustMethod() {
        return mRangingRobustMethod;
    }

    /**
     * Sets robust method used for robust position estimation using ranging data.
     * @param rangingRobustMethod robust method used for robust position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingRobustMethod(RobustEstimatorMethod rangingRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRangingRobustMethod = rangingRobustMethod;
    }

    /**
     * Gets robust method used for pathloss exponent and transmitted power estimation
     * using RSSI data.
     * @return robust method used for pathloss exponent and transmitted power
     * estimation.
     */
    public RobustEstimatorMethod getRssiRobustMethod() {
        return mRssiRobustMethod;
    }

    /**
     * Sets robust method used for pathloss exponent and transmitted power estimation
     * using RSSI data.
     * @param rssiRobustMethod robust method used for pathloss exponent and transmitted
     *                         power estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiRobustMethod(RobustEstimatorMethod rssiRobustMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRssiRobustMethod = rssiRobustMethod;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * position estimation.
     * If not defined, default threshold will be used.
     * @return threshold for ranging estimation or null.
     */
    public Double getRangingThreshold() {
        return mRangingThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * position estimation.
     * If not defined, default threshold will be used.
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
     * pathloss exponent and transmitted power estimation.
     * If not defined, default threshold will be used.
     * @return threshold for RSSI estimation or null.
     */
    public Double getRssiThreshold() {
        return mRssiThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * pathloss exponent and transmitted power estimation.
     * If not defined, default threshold will be used.
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
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%) for robust position estimation. The amount of
     * confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
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
     * @param rangingConfidence confidence to be set for robust position estimation
     *                          as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingConfidence(double rangingConfidence)
            throws IllegalArgumentException, LockedException {
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
     * @param rssiConfidence confidence to be set for robust pathloss exponent and
     *                       transmitted power estimation as a value between 0.0 and
     *                       1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiConfidence(double rssiConfidence)
            throws IllegalArgumentException, LockedException {
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
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRangingMaxIterations() {
        return mRangingMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust position estimation. When
     * the maximum number of iterations is exceeded, an approximate result might be
     * available for retrieval.
     * @param rangingMaxIterations maximum allowed number of iterations to be set
     *                             for position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setRangingMaxIterations(int rangingMaxIterations)
            throws IllegalArgumentException, LockedException {
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
     * @param rssiMaxIterations maximum allowed number of iterations to be set for
     *                          pathloss exponent and transmitted power estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setRssiMaxIterations(int rssiMaxIterations)
            throws IllegalArgumentException, LockedException {
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
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if estimator is locked.
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
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
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
     * Gets signal readings belonging to the same radio source.
     * @return signal readings belonging to the same radio source.
     */
    public List<? extends ReadingLocated<P>> getReadings() {
        return mReadings;
    }

    /**
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same
     *                 radio source.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public void setReadings(
            List<? extends ReadingLocated<P>> readings)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetReadings(readings);
    }

    /**
     * Gets listener in charge of attending events raised by this instance.
     * @return listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimatorListener<S, P> getListener() {
        return mListener;
    }

    /**
     * Sets listener in charge of attending events raised by this instance.
     * @param listener listener in charge of attending events raised by this
     *                 instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            SequentialRobustMixedRadioSourceEstimatorListener<S, P> listener)
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
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    public void setQualityScores(double[] qualityScores)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
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
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted
     *                                   power.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialTransmittedPowerdBm(Double initialTransmittedPowerdBm)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
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
     * @param initialTransmittedPower initial transmitted power to start the
     *                                estimation of radio source transmitted power.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setInitialTransmittedPower(Double initialTransmittedPower)
            throws LockedException, IllegalArgumentException {
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
     * @return initial position to start the estimation of radio source position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
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
     *
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
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
     *
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     * @param initialPathLossExponent initial path loss exponent.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPathLossExponent(double initialPathLossExponent)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     * @return true if transmitted power estimation is enabled, false otherwise.
     */
    public boolean isTransmittedPowerEstimationEnabled() {
        return mTransmittedPowerEstimationEnabled;
    }

    /**
     * Specifies whether transmitted power estimation is enabled or not.
     * @param transmittedPowerEstimationEnabled true if transmitted power estimation is enabled,
     *                                          false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setTransmittedPowerEstimationEnabled(boolean transmittedPowerEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mTransmittedPowerEstimationEnabled = transmittedPowerEstimationEnabled;
    }

    /**
     * Indicates whether path loss estimation is enabled or not.
     * @return true if path loss estimation is enabled, false otherwise.
     */
    public boolean isPathLossEstimationEnabled() {
        return mPathLossEstimationEnabled;
    }

    /**
     * Specifies whether path loss estimation is enabled or not.
     * @param pathLossEstimationEnabled true if path loss estimation is enabled,
     *                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPathLossEstimationEnabled(boolean pathLossEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPathLossEstimationEnabled = pathLossEstimationEnabled;
    }

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     * @return true to take into account reading position covariances, false otherwise.
     */
    public boolean getUseReadingPositionCovariance() {
        return mUseReadingPositionCovariances;
    }

    /**
     * Specifies whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     * @param useReadingPositionCovariances true to take into account reading position covariances, false
     *                                      otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseReadingPositionCovariances(boolean useReadingPositionCovariances)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseReadingPositionCovariances = useReadingPositionCovariances;
    }

    /**
     * Gets covariance for estimated position and power.
     * Matrix contains information in the following order:
     * Top-left submatrix contains covariance of position,
     * then follows transmitted power variance, and finally
     * the last element contains pathloss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     * @return covariance for estimated position and power.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated position covariance.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).*
     * This is only available when result has been refined and covariance is kept.
     * @return estimated position covariance.
     */
    public Matrix getEstimatedPositionCovariance() {
        return mEstimatedPositionCovariance;
    }

    /**
     * Gets estimated position.
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return mEstimatedPosition;
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    public boolean areValidReadings(
            List<? extends ReadingLocated<P>> readings) {
        if (readings == null) {
            return false;
        }

        checkReadings(readings);

        //if enough ranging data is available, we check validity both for ranging and RSSI readings
        return ((!mRssiPositionEnabled && mNumRangingReadings >= getMinRangingReadings() &&
                mNumRssiReadings >= getMinRssiReadings()) ||
                //if not enough ranging data is available, we check validity only for RSSI readings
                (mRssiPositionEnabled && mNumRssiReadings >= getMinRssiReadings()) ||
                //if only position is enabled, then only check for ranging readings
                (!mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled &&
                mNumRangingReadings >= getMinRangingReadings())) &&
                //in both upper cases enough general readings must be available
                readings.size() >= getMinReadings();
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     * @return true if this instance is ready, false otherwise.
     * @throws LockedException if estimator is locked
     */
    public boolean isReady() throws LockedException {
        checkReadings(mReadings);

        buildRangingEstimatorIfNeeded();
        setupRangingEstimator();

        if (mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled) {
            buildRssiEstimatorIfNeeded();
            setupRssiEstimator();
        }

        if (mRssiPositionEnabled) {
            return mRssiEstimator.isReady();
        } else {
            return mRangingEstimator.isReady() &&
                    ((!mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled) ||
                            mRssiEstimator.isReady());
        }
    }

    /**
     * Gets minimum required number of ranging or ranging+rssi readings
     * required to start estimation.
     * @return minimum required number of ranging or ranging+rssi readings.
     */
    public int getMinRangingReadings() {
        return getNumberOfDimensions() + 1;
    }

    /**
     * Gets minimum required number of rssi or ranging+rssi readings
     * required to start estimation.
     * @return minimum required number of rssi or ranging+rssi readings.
     */
    public int getMinRssiReadings() {
        return getMinReadings();
    }

    /**
     * Gets minimum required number of readings to estimate
     * power, position and pathloss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings for 2D, and 4 readings for 3D.
     * @return minimum required number of readings.
     */
    public abstract int getMinReadings();

    /**
     * Gets number of dimensions of position points.
     * @return number of dimensions of position points.
     */
    public abstract int getNumberOfDimensions();


    /**
     * Gets estimated transmitted power variance.
     * This is only available when result has been refined and covariance is kept.
     * @return estimated transmitted power variance.
     */
    public Double getEstimatedTransmittedPowerVariance() {
        return mEstimatedTransmittedPowerVariance;
    }

    /**
     * Gets estimated path loss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     * @return estimated path loss exponent variance.
     */
    public Double getEstimatedPathLossExponentVariance() {
        return mEstimatedPathLossExponentVariance;
    }

    /**
     * Gets estimated transmitted power expressed in milli watts (mW) or null if
     * not available.
     * @return estimated transmitted power expressed in milli watts or null.
     */
    public Double getEstimatedTransmittedPower() {
        return mEstimatedTransmittedPowerdBm != null ?
                Utils.dBmToPower(mEstimatedTransmittedPowerdBm) : null;
    }

    /**
     * Gets estimated transmitted power expressed in dBm's or null if not available.
     * @return estimated transmitted power expressed in dBm's or null.
     */
    public Double getEstimatedTransmittedPowerdBm() {
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
     * @return estimated path loss exponent.
     */
    public double getEstimatedPathLossExponent() {
        return mEstimatedPathLossExponent;
    }

    /**
     * Robustly estimates position, transmitted power and pathloss exponent for a
     * radio source.
     * @throws LockedException if instance is busy during estimation.
     * @throws NotReadyException if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
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
            if (!mRssiPositionEnabled) {
                mRangingEstimator.estimate();

                mEstimatedPosition = mRangingEstimator.getEstimatedPosition();
                mEstimatedPositionCovariance =
                        mRangingEstimator.getEstimatedPositionCovariance();
            } else {
                mEstimatedPosition = null;
            }

            //estimate transmitted power and/or pathloss if enabled
            if (mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled ||
                    mRssiPositionEnabled) {
                mRssiEstimator.setPositionEstimationEnabled(mRssiPositionEnabled);
                mRssiEstimator.setInitialPosition(mEstimatedPosition);

                mRssiEstimator.estimate();

                if (mRssiPositionEnabled) {
                    mEstimatedPosition = mRssiEstimator.getEstimatedPosition();
                    mEstimatedPositionCovariance =
                            mRssiEstimator.getEstimatedPositionCovariance();
                }

                if (mTransmittedPowerEstimationEnabled) {
                    //transmitted power estimation enabled
                    mEstimatedTransmittedPowerdBm =
                            mRssiEstimator.getEstimatedTransmittedPowerdBm();
                    mEstimatedTransmittedPowerVariance =
                            mRssiEstimator.getEstimatedTransmittedPowerVariance();
                } else {
                    //transmitted power estimation disabled
                    mEstimatedTransmittedPowerdBm = mInitialTransmittedPowerdBm;
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
                if (mRssiPositionEnabled) {
                    //if only RSSI estimation is done, we use directly the available estimated covariance
                    mCovariance = mRssiEstimator.getCovariance();
                } else {
                    //if both ranging and RSSI data is used, we build covariance matrix by setting
                    //position covariance estimated by ranging estimator into top-left corner, and then
                    //adding covariance terms related to pathloss exponent and transmitted power
                    Matrix rssiCov = mRssiEstimator.getCovariance();
                    if (mEstimatedPositionCovariance != null && rssiCov != null) {
                        int dims = getNumberOfDimensions();
                        int n = dims;
                        if (mTransmittedPowerEstimationEnabled) {
                            n++;
                        }
                        if (mPathLossEstimationEnabled) {
                            n++;
                        }

                        int dimsMinus1 = dims - 1;
                        int nMinus1 = n - 1;
                        mCovariance = new Matrix(n, n);
                        mCovariance.setSubmatrix(0, 0,
                                dimsMinus1, dimsMinus1,
                                mEstimatedPositionCovariance);
                        mCovariance.setSubmatrix(dims, dims,
                                nMinus1, nMinus1, rssiCov);
                    } else {
                        mCovariance = null;
                    }
                }
            } else {
                mCovariance = mEstimatedPositionCovariance;
                mEstimatedTransmittedPowerdBm = mInitialTransmittedPowerdBm;
                mEstimatedTransmittedPowerVariance = null;

                mEstimatedPathLossExponent = mInitialPathLossExponent;
                mEstimatedPathLossExponentVariance = null;
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } catch (AlgebraException e) {
            throw new RobustEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets estimated located radio source.
     * @return estimated located radio source.
     * @param <RS> type of located radio source.
     */
    public abstract <RS extends RadioSourceLocated<P>> RS getEstimatedRadioSource();

    /**
     * Builds ranging estimator.
     */
    protected abstract void buildRangingEstimatorIfNeeded();

    /**
     * build RSSI estimator.
     * @throws LockedException if estimator is locked.
     */
    protected abstract void buildRssiEstimatorIfNeeded() throws LockedException;

    /**
     * Setups ranging estimator.
     * @throws LockedException if estimator is locked.
     */
    protected void setupRangingEstimator() throws LockedException {
        if (mReadings != null && !mRssiPositionEnabled) {
            //build ranging readings
            List<RangingReadingLocated<S, P>> rangingReadings = new ArrayList<>();
            for (ReadingLocated<P> reading : mReadings) {
                if (reading instanceof RangingReadingLocated) {
                    rangingReadings.add((RangingReadingLocated<S, P>)reading);
                } else if (reading instanceof RangingAndRssiReadingLocated) {
                    rangingReadings.add(createRangingReading(
                            (RangingAndRssiReadingLocated<S, P>)reading));
                }
            }
            mRangingEstimator.setReadings(rangingReadings);

            if (mQualityScores != null && !rangingReadings.isEmpty()) {
                //build quality scores
                int numReadings = mReadings.size();
                int numRangingReadings = rangingReadings.size();
                double[] rangingQualityScores = new double[numRangingReadings];
                int pos = 0;
                for (int i = 0; i < numReadings; i++) {
                    ReadingLocated<P> reading = mReadings.get(i);
                    if (reading instanceof RangingReadingLocated ||
                            reading instanceof RangingAndRssiReadingLocated) {
                        rangingQualityScores[pos] = mQualityScores[i];
                        pos++;
                    }
                }

                mRangingEstimator.setQualityScores(rangingQualityScores);
            }
        }

        mRangingEstimator.setProgressDelta(2.0f * mProgressDelta);
        mRangingEstimator.setConfidence(mRangingConfidence);
        mRangingEstimator.setMaxIterations(mRangingMaxIterations);
        mRangingEstimator.setResultRefined(mRefineResult);
        mRangingEstimator.setCovarianceKept(mKeepCovariance);
        mRangingEstimator.setUseReadingPositionCovariances(
                mUseReadingPositionCovariances);

        mRangingEstimator.setInitialPosition(mInitialPosition);

        mRangingEstimator.setListener(new RobustRangingRadioSourceEstimatorListener<S, P>() {
            @Override
            public void onEstimateStart(RobustRangingRadioSourceEstimator<S, P> estimator) { }

            @Override
            public void onEstimateEnd(RobustRangingRadioSourceEstimator<S, P> estimator) { }

            @Override
            public void onEstimateNextIteration(RobustRangingRadioSourceEstimator<S, P> estimator,
                                                int iteration) { }

            @Override
            public void onEstimateProgressChange(RobustRangingRadioSourceEstimator<S, P> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            SequentialRobustMixedRadioSourceEstimator.this,
                            0.5f * progress);
                }
            }
        });
    }

    /**
     * Setups RSSI estimator.
     * @throws LockedException if estimator is locked.
     */
    protected void setupRssiEstimator() throws LockedException {
        if (mReadings != null) {
            mRssiEstimator.setPositionEstimationEnabled(mRssiPositionEnabled);

            //build RSSI readings
            List<RssiReadingLocated<S, P>> rssiReadings = new ArrayList<>();
            for (ReadingLocated<P> reading : mReadings) {
                if (reading instanceof RssiReadingLocated) {
                    rssiReadings.add((RssiReadingLocated<S, P>)reading);
                } else if (reading instanceof RangingAndRssiReadingLocated) {
                    rssiReadings.add(createRssiReading(
                            (RangingAndRssiReadingLocated<S, P>)reading));
                }
            }
            mRssiEstimator.setReadings(rssiReadings);

            if (mQualityScores != null && !rssiReadings.isEmpty()) {
                //build quality scores
                int numReadings = mReadings.size();
                int numRssiReadings = rssiReadings.size();
                double[] rssiQualityScores = new double[numRssiReadings];
                int pos = 0;
                for (int i = 0; i < numReadings; i++) {
                    ReadingLocated<P> reading = mReadings.get(i);
                    if (reading instanceof RssiReadingLocated ||
                            reading instanceof RangingAndRssiReadingLocated) {
                        rssiQualityScores[pos] = mQualityScores[i];
                        pos++;
                    }
                }

                mRssiEstimator.setQualityScores(rssiQualityScores);
            }
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
            public void onEstimateStart(RobustRssiRadioSourceEstimator<S, P> estimator) { }

            @Override
            public void onEstimateEnd(RobustRssiRadioSourceEstimator<S, P> estimator) { }

            @Override
            public void onEstimateNextIteration(RobustRssiRadioSourceEstimator<S, P> estimator,
                                                int iteration) { }

            @Override
            public void onEstimateProgressChange(RobustRssiRadioSourceEstimator<S, P> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            SequentialRobustMixedRadioSourceEstimator.this,
                            0.5f + 0.5f * progress);
                }
            }
        });
    }

    /**
     * Internally sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are null, not enough readings
     * are available, or readings do not belong to the same access point.
     */
    private void internalSetReadings(
            List<? extends ReadingLocated<P>> readings)
            throws IllegalArgumentException {
        if (!areValidReadings(readings)) {
            throw new IllegalArgumentException();
        }

        mReadings = readings;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than required minimum.
     */
    private void internalSetQualityScores(double[] qualityScores)
            throws IllegalArgumentException {
        if (qualityScores == null ||
                qualityScores.length < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }

    /**
     * Creates a ranging reading from a ranging and RSSI reading.
     * @param reading input reading to convert from.
     * @return a ranging reading containing only the ranging data of input reading.
     */
    private RangingReadingLocated<S, P> createRangingReading(RangingAndRssiReadingLocated<S, P> reading) {
        return new RangingReadingLocated<>(reading.getSource(),
                reading.getDistance(), reading.getPosition(),
                reading.getDistanceStandardDeviation(),
                reading.getPositionCovariance());
    }

    /**
     * Creates an RSSI reading from a ranging and RSSI reading.
     * @param reading input reading to convert from.
     * @return an RSSI reading containing only the RSSI data of input reading.
     */
    private RssiReadingLocated<S, P> createRssiReading(RangingAndRssiReadingLocated<S, P> reading) {
        return new RssiReadingLocated<>(reading.getSource(),
                reading.getRssi(), reading.getPosition(),
                reading.getRssiStandardDeviation(),
                reading.getPositionCovariance());
    }

    /**
     * Checks number of available ranging readings and number of available RSSI readings. Also determines
     * whether position must be estimated using ranging data or RSSI data.
     * @param readings readings to be checked.
     */
    private void checkReadings(List<? extends ReadingLocated<P>> readings) {
        mNumRangingReadings = mNumRssiReadings = 0;

        if (readings == null) {
            return;
        }

        for (ReadingLocated<P> reading : readings) {
            if (reading instanceof RangingReadingLocated) {
                mNumRangingReadings++;

            } else if (reading instanceof RssiReadingLocated) {
                mNumRssiReadings++;

            } else if (reading instanceof RangingAndRssiReadingLocated) {
                mNumRangingReadings++;
                mNumRssiReadings++;
            }
        }

        mRssiPositionEnabled = mNumRangingReadings < getMinRangingReadings();
    }
}
