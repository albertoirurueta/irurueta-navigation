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
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.NavigationException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 2D position, transmitted power and pathloss
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
 * Because usually information about the antena of the radio source cannot be
 * retrieved (because many measurements are made on unkown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 *
 * Althouth RobustRssiRadioSourceEstimator can estimate the same parameters of a radio
 * source, when ranging measures are available along with RSSI measurements,
 * implementations of this class should be preferred instead as they can provide
 * greater accuracy.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustRangingAndRssiRadioSourceEstimator2D<S extends RadioSource> extends
        RobustRangingAndRssiRadioSourceEstimator<S, Point2D> {

    /**
     * Radio source estimator used internally.
     */
    protected RangingAndRssiRadioSourceEstimator2D<S> mInnerEstimator =
            new RangingAndRssiRadioSourceEstimator2D<>();

    /**
     * Subset of readings used by inner estimator.
     */
    private List<RangingAndRssiReadingLocated<S, Point2D>> mInnerReadings =
            new ArrayList<>();

    /**
     * Constructor.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets signal readings bleonging to the same radio source.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition)
            throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings WiFi signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     * @param readings WiFi signal readings belonging to the radio source point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
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
    public RobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
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
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent)
            throws IllegalArgumentException {
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
    public RobustRangingAndRssiRadioSourceEstimator2D(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm,
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
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
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
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingAndRssiRadioSourceEstimator2D(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>();
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>();
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>();
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>();
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D radio source power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D radio source power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>();
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>();
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, initialTransmittedPowerdBm,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition, initialTransmittedPowerdBm,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case LMedS:
                return new LMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case MSAC:
                return new MSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        readings, initialPosition, initialTransmittedPowerdBm,
                        initialPathLossExponent, listener);
            case PROSAC:
                return new PROSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingAndRssiRadioSourceEstimator2D<>(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, initialPathLossExponent,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        return create(readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) throws IllegalArgumentException {
        return create(readings, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition) {
        return create(initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(readings, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Double initialTransmittedPowerdBm) {
        return create(initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        return create(readings, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(readings, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm) {
        return create(initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(initialPosition, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        return create(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        return create(qualityScores, readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition) {
        return create(qualityScores, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(qualityScores, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        return create(qualityScores, readings, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(qualityScores, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, initialPathLossExponent,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent) {
        return create(qualityScores, initialPosition,
                initialTransmittedPowerdBm, initialPathLossExponent,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position radio source estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 2D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     * @param <S> a {@link RadioSource} type.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, initialPathLossExponent, listener,
                DEFAULT_ROBUST_METHOD);
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
        return mInnerEstimator.getNumberOfDimensions();
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
     * Solves preliminar solution for a subset of samples.
     * @param samplesIndices indices of subset samples.
     * @param solutions instance where solution will be stored.
     */
    @Override
    protected void solvePreliminarSolutions(int[] samplesIndices,
            List<Solution<Point2D>> solutions) {

        try {
            int index;

            mInnerReadings.clear();
            for (int samplesIndice : samplesIndices) {
                index = samplesIndice;
                mInnerReadings.add(mReadings.get(index));
            }

            //initial transmitted power and position might or might not be available
            mInnerEstimator.setInitialTransmittedPowerdBm(
                    mInitialTransmittedPowerdBm);
            mInnerEstimator.setInitialPosition(mInitialPosition);
            mInnerEstimator.setInitialPathLossExponent(mInitialPathLossExponent);

            mInnerEstimator.setTransmittedPowerEstimationEnabled(
                    mTransmittedPowerEstimationEnabled);
            mInnerEstimator.setPathLossEstimationEnabled(mPathLossEstimationEnabled);

            mInnerEstimator.setReadings(mInnerReadings);

            //indicastes whether readings position covariances must be taken into account
            mInnerEstimator.setUseReadingPositionCovariances(
                    mUseReadingPositionCovariances);

            mInnerEstimator.estimate();

            Point2D estimatedPosition = mInnerEstimator.getEstimatedPosition();
            double estimatedTransmittedPowerdBm =
                    mInnerEstimator.getEstimatedTransmittedPowerdBm();
            double estimatedPathLossExponent =
                    mInnerEstimator.getEstimatedPathLossExponent();
            solutions.add(new Solution<>(estimatedPosition,
                    estimatedTransmittedPowerdBm, estimatedPathLossExponent));
        } catch(NavigationException ignore) {
            //if anything fails, no solution is added
        }
    }

    /**
     * Attempts to refine estimated position and transmitted power contained in
     * provided solution if refinement is requested.
     * This method sets a refined result and transmitted power or provided input
     * result if refinement is not requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined result.
     * solution if not requested or refinement failed.
     * @param result result to be refined.
     */
    protected void attemptRefine(Solution<Point2D> result) {
        Point2D initialPosition = result.getEstimatedPosition();
        double initialTransmittedPowerdBm =
                result.getEstimatedTransmittedPowerdBm();
        double initialPathLossExponent = result.getEstimatedPathLossExponent();

        if (mRefineResult && mInliersData != null) {
            BitSet inliers = mInliersData.getInliers();
            int nSamples = mReadings.size();

            mInnerReadings.clear();

            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    //sample is inlier
                    mInnerReadings.add(mReadings.get(i));
                }
            }

            try {
                mInnerEstimator.setInitialPosition(initialPosition);
                mInnerEstimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
                mInnerEstimator.setInitialPathLossExponent(initialPathLossExponent);
                mInnerEstimator.setTransmittedPowerEstimationEnabled(
                        mTransmittedPowerEstimationEnabled);
                mInnerEstimator.setPathLossEstimationEnabled(
                        mPathLossEstimationEnabled);
                mInnerEstimator.setReadings(mInnerReadings);
                mInnerEstimator.setUseReadingPositionCovariances(
                        mUseReadingPositionCovariances);

                mInnerEstimator.estimate();

                Matrix cov = mInnerEstimator.getEstimatedCovariance();
                if (mKeepCovariance && cov != null) {
                    //keep covariance
                    mCovariance = cov;

                    int dims = getNumberOfDimensions();
                    int pos = 0;

                    int d = dims -1;
                    if (mEstimatedPositionCovariance == null) {
                        mEstimatedPositionCovariance = mCovariance.
                                getSubmatrix(0, 0, d, d);
                    } else {
                        mCovariance.getSubmatrix(0, 0, d, d,
                                mEstimatedPositionCovariance);
                    }
                    pos += dims;

                    if (mTransmittedPowerEstimationEnabled) {
                        //transmitted power estimation enabled
                        mEstimatedTransmittedPowerVariance = mCovariance.
                                getElementAt(pos, pos);
                        pos++;
                    } else {
                        //transmitted power estimation disabled
                        mEstimatedTransmittedPowerVariance = null;
                    }

                    if (mPathLossEstimationEnabled) {
                        //pathloss exponent estimation enabled
                        mEstimatedPathLossExponentVariance = mCovariance.
                                getElementAt(pos, pos);
                    } else {
                        //pathloss exponent estimation disabled
                        mEstimatedPathLossExponentVariance = null;
                    }
                } else {
                    mCovariance = null;
                    mEstimatedPositionCovariance = null;
                    mEstimatedTransmittedPowerVariance = null;
                    mEstimatedPathLossExponentVariance = null;
                }

                mEstimatedPosition = mInnerEstimator.getEstimatedPosition();
                mEstimatedTransmittedPowerdBm =
                        mInnerEstimator.getEstimatedTransmittedPowerdBm();
                mEstimatedPathLossExponent =
                        mInnerEstimator.getEstimatedPathLossExponent();
            } catch (Exception e) {
                //refinement failed, so we return input value, and covariance
                //becomes unavailable
                mCovariance = null;
                mEstimatedPositionCovariance = null;
                mEstimatedTransmittedPowerVariance = null;
                mEstimatedPathLossExponentVariance = null;

                mEstimatedPosition = initialPosition;
                mEstimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
                mEstimatedPathLossExponent = initialPathLossExponent;
            }
        } else {
            mCovariance = null;
            mEstimatedPositionCovariance = null;
            mEstimatedTransmittedPowerVariance = null;
            mEstimatedPathLossExponentVariance = null;

            mEstimatedPosition = initialPosition;
            mEstimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
            mEstimatedPathLossExponent = initialPathLossExponent;
        }
    }
}
