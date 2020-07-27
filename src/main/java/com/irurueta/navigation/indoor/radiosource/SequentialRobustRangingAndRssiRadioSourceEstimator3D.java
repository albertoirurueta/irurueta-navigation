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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Robustly estimates 3D position, transmitted power and pathloss exponent of a radio
 * source (e.g. WiFi access point or bluetooth beacon), by discarging
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
 * <p>
 * Implementations of this class sequentially estimate position and then remaining
 * parameters. First ranging data is used to robustly estimate position and then
 * remaining parameters are robustly estimated using former estimated position as
 * an initial guess.
 * <p>
 * Because usually information about the antena of the radio source cannot be
 * retrieved (because many measurements are made on unkown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 * <p>
 * Implementations of this class might produce more stable positions of estimated
 * radio sources than implementations of RobustRangingAndRssiRadioSourceEstimator3D.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class SequentialRobustRangingAndRssiRadioSourceEstimator3D<S extends RadioSource> extends
        SequentialRobustRangingAndRssiRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D() {
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings) {
        super(readings);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition) {
        super(readings, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition) {
        super(initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   in charge of attenging events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores) {
        super(qualityScores);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings) {
        super(qualityScores, readings);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, readings, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition) {
        super(qualityScores, readings, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition) {
        super(qualityScores, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, readings, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, readings, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator3D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }


    /**
     * Gets minimum required number of readings to estimate
     * power, position and pathloss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        int minReadings = Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
        if (isTransmittedPowerEstimationEnabled()) {
            minReadings++;
        }
        if (isPathLossEstimationEnabled()) {
            minReadings++;
        }
        return ++minReadings;
    }

    /**
     * Gets number of dimensions of position points.
     *
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     *
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceWithPowerAndLocated<Point3D> getEstimatedRadioSource() {
        final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        final S source = readings.get(0).getSource();

        final Point3D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        final Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        final Double transmittedPowerVariance =
                getEstimatedTransmittedPowerVariance();
        final Double transmittedPowerStandardDeviation = transmittedPowerVariance != null ?
                Math.sqrt(transmittedPowerVariance) : null;

        final Double pathlossExponentVariance =
                getEstimatedPathLossExponentVariance();
        final Double pathlossExponentStandardDeviation = pathlossExponentVariance != null ?
                Math.sqrt(pathlossExponentVariance) : null;

        if (source instanceof WifiAccessPoint) {
            final WifiAccessPoint accessPoint = (WifiAccessPoint) source;
            return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(),
                    source.getFrequency(), accessPoint.getSsid(),
                    getEstimatedTransmittedPowerdBm(),
                    transmittedPowerStandardDeviation,
                    getEstimatedPathLossExponent(),
                    pathlossExponentStandardDeviation,
                    estimatedPosition,
                    estimatedPositionCovariance);
        } else if (source instanceof Beacon) {
            final Beacon beacon = (Beacon) source;
            return new BeaconWithPowerAndLocated3D(beacon.getIdentifiers(),
                    getEstimatedTransmittedPowerdBm(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(),
                    getEstimatedPathLossExponent(),
                    transmittedPowerStandardDeviation,
                    pathlossExponentStandardDeviation,
                    estimatedPosition, estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Builds ranging estimator.
     */
    @Override
    protected void buildRangingEstimatorIfNeeded() {
        if (mRangingEstimator == null || mRangingEstimator.getMethod() != mRangingRobustMethod) {
            mRangingEstimator = RobustRangingRadioSourceEstimator3D.create(mRangingRobustMethod);
        }
    }

    /**
     * Build RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void buildRssiEstimatorIfNeeded() throws LockedException {
        if (mRssiEstimator == null || mRssiEstimator.getMethod() != mRssiRobustMethod) {
            mRssiEstimator = RobustRssiRadioSourceEstimator3D.create(mRssiRobustMethod);

            // rssi estimator will never need position estimator, but to
            // ensure it is ready we need to provide an initial position
            mRssiEstimator.setPositionEstimationEnabled(false);
            mRssiEstimator.setInitialPosition(Point3D.create());
        }
    }

    /**
     * Setups ranging estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRangingEstimator() throws LockedException {
        super.setupRangingEstimator();

        switch (mRangingRobustMethod) {
            case RANSAC:
                ((RANSACRobustRangingRadioSourceEstimator3D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case LMedS:
                ((LMedSRobustRangingRadioSourceEstimator3D<S>) mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRangingRadioSourceEstimator3D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                MSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRangingRadioSourceEstimator3D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case PROMedS:
                ((PROMedSRobustRangingRadioSourceEstimator3D<S>) mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD);
                break;
            default:
                break;
        }
    }

    /**
     * Setups RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRssiEstimator() throws LockedException {
        super.setupRssiEstimator();

        switch (mRssiRobustMethod) {
            case RANSAC:
                ((RANSACRobustRssiRadioSourceEstimator3D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                RANSACRobustRssiRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case LMedS:
                ((LMedSRobustRssiRadioSourceEstimator3D<S>) mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRssiRadioSourceEstimator3D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                MSACRobustRssiRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRssiRadioSourceEstimator3D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROSACRobustRssiRadioSourceEstimator3D.DEFAULT_THRESHOLD);
                break;
            case PROMedS:
                ((PROMedSRobustRssiRadioSourceEstimator3D<S>) mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD);
                break;
            default:
                break;
        }
    }
}
