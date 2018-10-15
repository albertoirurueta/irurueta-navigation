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
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;

import java.util.List;

/**
 * Robustly estimate 3D position, transmitted power and pathloss exponent of a radio
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
 * This implementation is like SequentialRobustRangingAndRssiRadioSourceEstimator but
 * allows mixing different kinds of located radio source readings (ranging, RSSI
 * and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class SequentialRobustMixedRadioSourceEstimator3D<S extends RadioSource> extends
        SequentialRobustMixedRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public SequentialRobustMixedRadioSourceEstimator3D() { }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator3D(
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustMixedRadioSourceEstimator3D(
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm)
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
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition,
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
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition,
                                                       Double initialTransmittedPowerdBm,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
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
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition,
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
    public SequentialRobustMixedRadioSourceEstimator3D(Point3D initialPosition,
                                                       Double initialTransmittedPowerdBm, double initialPathLossExponent,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       Point3D initialPosition) throws IllegalArgumentException {
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores, Point3D initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores,
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm)
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
    public SequentialRobustMixedRadioSourceEstimator3D(
            double[] qualityScores, Point3D initialPosition,
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       List<? extends ReadingLocated<Point3D>> readings,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       List<? extends ReadingLocated<Point3D>> readings,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
                                                       double initialPathLossExponent,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
    public SequentialRobustMixedRadioSourceEstimator3D(double[] qualityScores,
                                                       List<? extends ReadingLocated<Point3D>> readings,
                                                       Point3D initialPosition, Double initialTransmittedPowerdBm,
                                                       double initialPathLossExponent,
                                                       SequentialRobustMixedRadioSourceEstimatorListener<S, Point3D> listener)
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
        int minReadings = Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
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
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceLocated<Point3D> getEstimatedRadioSource() {
        List<? extends ReadingLocated<Point3D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }

        S source;
        ReadingLocated<Point3D> reading = readings.get(0);
        if (reading instanceof RangingReadingLocated) {
            source = ((RangingReadingLocated<S, Point3D>)reading).getSource();
        } else if (reading instanceof RssiReadingLocated) {
            source = ((RssiReadingLocated<S, Point3D>)reading).getSource();
        } else if (reading instanceof RangingAndRssiReadingLocated) {
            source = ((RangingAndRssiReadingLocated<S, Point3D>)reading).getSource();
        } else {
            return null;
        }

        Point3D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        Double transmittedPowerdBm = getEstimatedTransmittedPowerdBm();

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
            if (transmittedPowerdBm != null) {
                return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(),
                        source.getFrequency(), accessPoint.getSsid(),
                        transmittedPowerdBm,
                        transmittedPowerStandardDeviation,
                        getEstimatedPathLossExponent(),
                        pathlossExponentStandardDeviation,
                        estimatedPosition,
                        estimatedPositionCovariance);
            } else {
                return new WifiAccessPointLocated3D(accessPoint.getBssid(),
                        source.getFrequency(), accessPoint.getSsid(),
                        estimatedPosition, estimatedPositionCovariance);
            }
        } else if(source instanceof Beacon) {
            Beacon beacon = (Beacon) source;
            return new BeaconWithPowerAndLocated3D(beacon.getIdentifiers(),
                    beacon.getTransmittedPower(), beacon.getFrequency(),
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
            mRangingEstimator = RobustRangingRadioSourceEstimator3D.create(mRangingRobustMethod);
        }
    }

    /**
     * build RSSI estimator.
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void buildRssiEstimatorIfNeeded() throws LockedException {
        if (mRssiEstimator == null || mRssiEstimator.getMethod() != mRssiRobustMethod) {
            mRssiEstimator = RobustRssiRadioSourceEstimator3D.create(mRssiRobustMethod);

            //rssi estimator will never need position estimator, but to
            //ensure it is ready we need to provide an initial position
            mRssiEstimator.setPositionEstimationEnabled(false);
            mRssiEstimator.setInitialPosition(Point3D.create());
        }
    }
}
