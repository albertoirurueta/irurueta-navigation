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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;

import java.util.List;

/**
 * Robustly estimates 2D position, transmitted power and pathloss exponent of a radio
 * source (e.g. WiFi access point or bluetooth beacon), by discarding
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
 * Implementations of this class might produce more stable positions of estimated
 * radio sources than implementations of RobustRangingAndRssiRadioSourceEstimator2D.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class SequentialRobustRangingAndRssiRadioSourceEstimator2D<S extends RadioSource> extends
        SequentialRobustRangingAndRssiRadioSourceEstimator<S, Point2D> {

    /**
     * Constructor.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D() { }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @throws IllegalArgumentException if quality scores is null, or length of
     * quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores)
            throws IllegalArgumentException {
        super(qualityScores);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        super(qualityScores, readings);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, readings, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) throws IllegalArgumentException {
        super(qualityScores, readings, initialPosition);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            Point2D initialPosition) throws IllegalArgumentException {
        super(qualityScores, initialPosition);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores, Point2D initialPosition,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, initialPosition, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores,readings, initialPosition, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(qualityScores, initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        super(qualityScores, readings, initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, readings, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }


    /**
     * Gets minimum required number of readings to estimate
     * power, position and pathloss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings.
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        int minReadings = Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
        if (isTransmittedPowerEstimationEnabled()) {
            minReadings++;
        }
        if (isPathLossEstimationEnabled()) {
            minReadings ++;
        }
        return ++minReadings;
    }

    /**
     * Gets number of dimensions of position points.
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceWithPowerAndLocated<Point2D> getEstimatedRadioSource() {
        List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        S source = readings.get(0).getSource();

        Point2D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        Double transmittedPowerVariance =
                getEstimatedTransmittedPowerVariance();
        Double transmittedPowerStandardDeviation = transmittedPowerVariance != null ?
                Math.sqrt(transmittedPowerVariance) : null;

        Double pathlossExponentVariance =
                getEstimatedPathLossExponentVariance();
        Double pathlossExponentStandardDeviation = pathlossExponentVariance != null ?
                Math.sqrt(pathlossExponentVariance) : null;

        if (source instanceof WifiAccessPoint) {
            WifiAccessPoint accessPoint = (WifiAccessPoint) source;
            return new WifiAccessPointWithPowerAndLocated2D(accessPoint.getBssid(),
                    source.getFrequency(), accessPoint.getSsid(),
                    getEstimatedTransmittedPowerdBm(),
                    transmittedPowerStandardDeviation,
                    getEstimatedPathLossExponent(),
                    pathlossExponentStandardDeviation,
                    estimatedPosition,
                    estimatedPositionCovariance);
        } else if(source instanceof Beacon) {
            Beacon beacon = (Beacon) source;
            return new BeaconWithPowerAndLocated2D(beacon.getIdentifiers(),
                    getEstimatedTransmittedPowerdBm(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(),
                    getEstimatedPathLossExponent(),
                    transmittedPowerStandardDeviation,
                    pathlossExponentStandardDeviation,
                    estimatedPosition, estimatedPositionCovariance);
        }else {
            return null;
        }
    }

    /**
     * Builds ranging estimator.
     */
    @Override
    protected void buildRangingEstimatorIfNeeded() {
        if (mRangingEstimator == null || mRangingEstimator.getMethod() != mRangingRobustMethod) {
            mRangingEstimator = RobustRangingRadioSourceEstimator2D.create(mRangingRobustMethod);
        }
    }

    /**
     * build RSSI estimator.
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void buildRssiEstimatorIfNeeded() throws LockedException {
        if (mRssiEstimator == null || mRssiEstimator.getMethod() != mRssiRobustMethod) {
            mRssiEstimator = RobustRssiRadioSourceEstimator2D.create(mRssiRobustMethod);

            //rssi estimator will never need position estimator, but to
            //ensure it is ready we need to provide an initial position
            mRssiEstimator.setPositionEstimationEnabled(false);
            mRssiEstimator.setInitialPosition(Point2D.create());
        }
    }

    /**
     * Setups ranging estimator.
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRangingEstimator() throws LockedException {
        super.setupRangingEstimator();

        switch (mRangingRobustMethod) {
            case RANSAC:
                ((RANSACRobustRangingRadioSourceEstimator2D<S>)mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                RANSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case LMedS:
                ((LMedSRobustRangingRadioSourceEstimator2D<S>)mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                LMedSRobustRangingRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRangingRadioSourceEstimator2D<S>)mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                MSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRangingRadioSourceEstimator2D<S>)mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROMedS:
                ((PROMedSRobustRangingRadioSourceEstimator2D<S>)mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROMedSRobustRangingRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
        }
    }

    /**
     * Setups RSSI estimator.
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRssiEstimator() throws LockedException {
        super.setupRssiEstimator();

        switch (mRssiRobustMethod) {
            case RANSAC:
                ((RANSACRobustRssiRadioSourceEstimator2D<S>)mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                RANSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case LMedS:
                ((LMedSRobustRssiRadioSourceEstimator2D<S>)mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                LMedSRobustRssiRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRssiRadioSourceEstimator2D<S>)mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                MSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRssiRadioSourceEstimator2D<S>)mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROMedS:
                ((PROMedSRobustRssiRadioSourceEstimator2D<S>)mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROMedSRobustRssiRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
        }
    }
}
