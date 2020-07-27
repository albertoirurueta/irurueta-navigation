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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly estimate 3D position, transmitted power and pathloss exponent of a radio source
 * (e.g. WiFi access point or bluetooth beacon), by discarding outliers using PROMedS
 * algorithm and assuming that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If RssiReadings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * Implementations of this class should be able to detect and discard outliers in
 * order to find the best solution.
 * <p>
 * IMPORTANT: When using this class estimation can be done using a
 * combination of radio source position, transmitted power and path loss
 * exponent. However enabling all three estimations usually achieves
 * inaccurate results. When using this class, estimation must be of at least
 * one parameter (position, transmitted power or path loss exponent) when
 * initial values are provided for the other two, and at most it should consist
 * of two parameters (either position and transmitted power, position and
 * path loss exponent or transmitted power and path loss exponent), providing an
 * initial value for the remaining parameter.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("Duplicates")
public class PROMedSRobustRssiRadioSourceEstimator3D<S extends RadioSource> extends
        RobustRssiRadioSourceEstimator3D<S> {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-4;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
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
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores) {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings) {
        super(readings);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition) {
        super(readings, initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores, final Point3D initialPosition) {
        super(initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores, final Point3D initialPosition,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
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
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
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
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
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
     * @param initialPosition            initial position to start the estimation of access
     *                                   point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings containing to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
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
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
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
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm,
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
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
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
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROMedSRobustRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
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
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
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
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = stopThreshold;
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
    @Override
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
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mReadings.size();
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
    @Override
    public void estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROMedSRobustEstimator<Solution<Point3D>> innerEstimator =
                new PROMedSRobustEstimator<>(
                        new PROMedSRobustEstimatorListener<Solution<Point3D>>() {

                            @Override
                            public double[] getQualityScores() {
                                return mQualityScores;
                            }

                            @Override
                            public double getThreshold() {
                                return mStopThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mReadings.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return Math.max(mPreliminarySubsetSize, getMinReadings());
                            }

                            @Override
                            public void estimatePreliminarSolutions(
                                    final int[] samplesIndices,
                                    final List<Solution<Point3D>> solutions) {
                                solvePreliminarSolutions(samplesIndices, solutions);
                            }

                            @Override
                            public double computeResidual(
                                    final Solution<Point3D> currentEstimation,
                                    final int i) {
                                return residual(currentEstimation, i);
                            }

                            @Override
                            public boolean isReady() {
                                return PROMedSRobustRssiRadioSourceEstimator3D.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<Solution<Point3D>> estimator) {
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<Solution<Point3D>> estimator) {
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<Solution<Point3D>> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROMedSRobustRssiRadioSourceEstimator3D.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<Solution<Point3D>> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROMedSRobustRssiRadioSourceEstimator3D.this,
                                            progress);
                                }
                            }
                        });

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            mInliersData = null;

            //inlier thresholds are disable to obtain a less restrictive amount of inliers
            innerEstimator.setUseInlierThresholds(false);

            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final Solution<Point3D> result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            attemptRefine(result);

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
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
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null ||
                qualityScores.length < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
