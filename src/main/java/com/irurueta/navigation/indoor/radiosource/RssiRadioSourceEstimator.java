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
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;

import java.util.List;

/**
 * Estimates position, transmitted power and path loss exponent of a
 * radio source (e.g. WiFi access point or bluetooth beacon) assuming
 * that the radio source emits isotropically following the expression
 * below:
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
 * <p>
 * IMPORTANT: Implementations of this class can choose to estimate a
 * combination of radio source position, transmitted power and path loss
 * exponent. However enabling all three estimations usually achieves
 * innacurate results. When using this class, estimation must be of at least
 * one parameter (position, transmitted power or path loss exponent) when
 * initial values are provided for the other two, and at most it should consist
 * of two parameters (either position and transmitted power, position and
 * path loss exponent or transmitted power and path loss exponent), providing an
 * initial value for the remaining parameter.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RssiRadioSourceEstimator<S extends RadioSource, P extends Point<P>>
        extends RadioSourceEstimator<P, RssiReadingLocated<S, P>,
        RssiRadioSourceEstimatorListener<S, P>> {

    /**
     * Speed of light expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = Constants.SPEED_OF_LIGHT;

    /**
     * Default standard deviations assumed for RSSI readings being fitted.
     */
    public static final double DEFAULT_POWER_STANDARD_DEVIATION = 1.0;

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Indicates whether radio source position estimation is enabled or not by default.
     */
    public static final boolean DEFAULT_POSITION_ESTIMATION_ENABLED = true;

    /**
     * Indicates whether radio source transmitted power estimation is enabled or not by
     * default. Typically this data is required for WiFi Access points, but it is already
     * provided for Beacons (and hence its estimation is not needed).
     */
    public static final boolean DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED = true;

    /**
     * Indicates whether path loss estimation is enabled or not by default.
     */
    public static final boolean DEFAULT_PATHLOSS_ESTIMATION_ENABLED = false;


    /**
     * Indicates whether radio source position estimation is enabled or not.
     */
    private boolean mPositionEstimationEnabled = DEFAULT_POSITION_ESTIMATION_ENABLED;

    /**
     * Estimated transmitted power expressed in dBm's.
     */
    private double mEstimatedTransmittedPowerdBm;

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     */
    private boolean mTransmittedPowerEstimationEnabled = DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED;

    /**
     * Estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     */
    private double mEstimatedPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

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
     * Estimated chi square value.
     */
    private double mChiSq;

    /**
     * Initial transmitted power to start the estimation of radio source
     * transmitted power.
     * If not defined, average value of received power readings will be used.
     */
    private Double mInitialTransmittedPowerdBm;

    /**
     * Initial position to start the estimation of radio source position.
     * If not defined, centroid of provided readings will be used.
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
    private double mInitialPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Indicates whether path loss estimation is enabled or not.
     */
    private boolean mPathLossEstimationEnabled = DEFAULT_PATHLOSS_ESTIMATION_ENABLED;

    /**
     * Levenberg-Marquardt fitter to find a solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter mFitter =
            new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Constructor.
     */
    public RssiRadioSourceEstimator() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator(
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public RssiRadioSourceEstimator(
            final P initialPosition) {
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final P initialPosition) {
        super(readings);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator(
            final P initialPosition,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public RssiRadioSourceEstimator(
            final Double initialTransmittedPowerdBm) {
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator(
            final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings);
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
     *                                   (expressed in dBm's)
     */
    public RssiRadioSourceEstimator(
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
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        mInitialPosition = initialPosition;
        mInitialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
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
    public RssiRadioSourceEstimator(
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
     *                                   (expressed in dBm's)
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator(
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final P initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        mInitialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will converte the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will converte the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted
     *                                   power.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialTransmittedPowerdBm(final Double initialTransmittedPowerdBm)
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will converte the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will converte the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * Gets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided readings will be used.
     * <p>
     * If position estimation is enabled, estimation will start at this value
     * and will converge to the most appropriate value.
     * If position estimation is disabled, this value will be assumed to
     * be exact and the estimated position will be equal to this value.
     *
     * @return initial position to start the estimation of radio source position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     * <p>
     * If position estimation is enabled, estimation will start at this value
     * and will converge to the most appropriate value.
     * If position estimation is disabled, this value will be assumed to
     * be exact and the estimated position will be equal to this value.
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
     * Indicates whether radio source position estimation is enabled or not.
     *
     * @return true if position estimation is enabled, false otherwise.
     */
    public boolean isPositionEstimationEnabled() {
        return mPositionEstimationEnabled;
    }

    /**
     * Specifies whether radio source position estimation is enabled or not.
     *
     * @param positionEstimationEnabled true if position estimation is enabled,
     *                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPositionEstimationEnabled(
            final boolean positionEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPositionEstimationEnabled = positionEstimationEnabled;
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
    public void setInitialPathLossExponent(
            final double initialPathLossExponent)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPathLossExponent = initialPathLossExponent;
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
            final boolean pathLossEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPathLossEstimationEnabled = pathLossEstimationEnabled;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        //at least one parameter estimtion must be enabled
        return (mPositionEstimationEnabled || mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled) &&
                //if position estimation is disabled, an initial position must be provided
                !(!mPositionEstimationEnabled && mInitialPosition == null) &&
                //if transmitted power estimation is disabled, an initial transmitted power must be provided
                !(!mTransmittedPowerEstimationEnabled && mInitialTransmittedPowerdBm == null) &&
                //readings must also be valid
                areValidReadings(mReadings);
    }

    /**
     * Estimate position, transmitted power and path loss exponent.
     *
     * @throws RadioSourceEstimationException if estimation fails.
     * @throws NotReadyException              if estimator is not ready.
     * @throws LockedException                if estimator is locked.
     */
    @SuppressWarnings("all")
    @Override
    public void estimate() throws RadioSourceEstimationException, NotReadyException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            if (mPositionEstimationEnabled &&
                    !mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled) {
                //only position estimation is enabled
                setupFitterPosition();
            } else if (!mPositionEstimationEnabled &&
                    mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled) {
                //only transmitted power estimation is enabled
                setupFitterTransmittedPower();
            } else if (!mPositionEstimationEnabled &&
                    !mTransmittedPowerEstimationEnabled && mPathLossEstimationEnabled) {
                //only pathloss estimation is enabled
                setupFitterPathLossExponent();
            } else if (mPositionEstimationEnabled &&
                    mTransmittedPowerEstimationEnabled && !mPathLossEstimationEnabled) {
                //position and transmitted power enabled
                setupFitterPositionAndTransmittedPower();
            } else if (mPositionEstimationEnabled &&
                    !mTransmittedPowerEstimationEnabled && mPathLossEstimationEnabled) {
                //position and pathloss enabled
                setupFitterPositionAndPathLossExponent();
            } else if (!mPositionEstimationEnabled &&
                    mTransmittedPowerEstimationEnabled && mPathLossEstimationEnabled) {
                //transmitted power and pathloss enabled
                setupFitterTransmittedPowerAndPathLossExponent();
            } else {
                //position, transmitted power and pathloss enabled
                setupFitterPositionTransmittedPowerAndPathLossExponent();
            }

            mFitter.fit();

            //estimated position and transmitted power
            final double[] a = mFitter.getA();
            final int dims = getNumberOfDimensions();

            mEstimatedCovariance = mFitter.getCovar();
            mChiSq = mFitter.getChisq();

            int pos = 0;
            mEstimatedPositionCoordinates = new double[dims];
            if (mPositionEstimationEnabled) {
                //position estimation enabled
                System.arraycopy(a, 0, mEstimatedPositionCoordinates, 0, dims);

                if (mEstimatedCovariance != null) {
                    final int d = dims - 1;
                    if (mEstimatedPositionCovariance == null) {
                        mEstimatedPositionCovariance = mEstimatedCovariance.
                                getSubmatrix(0, 0, d, d);
                    } else {
                        mEstimatedCovariance.getSubmatrix(0, 0, d, d,
                                mEstimatedPositionCovariance);
                    }
                }
                pos += dims;
            } else {
                //position estimation disabled
                if (mInitialPosition != null) {
                    for (int i = 0; i < dims; i++) {
                        mEstimatedPositionCoordinates[i] =
                                mInitialPosition.getInhomogeneousCoordinate(i);
                    }
                }

                mEstimatedPositionCovariance = null;
            }

            if (mTransmittedPowerEstimationEnabled) {
                //transmitted power estimation enabled
                mEstimatedTransmittedPowerdBm = a[pos];

                if (mEstimatedCovariance != null) {
                    mEstimatedTransmittedPowerVariance = mEstimatedCovariance.
                            getElementAt(pos, pos);
                }
                pos++;
            } else {
                //transmitted power estimation disabled
                if (mInitialTransmittedPowerdBm != null) {
                    mEstimatedTransmittedPowerdBm = mInitialTransmittedPowerdBm;
                }
                mEstimatedTransmittedPowerVariance = null;
            }

            if (mPathLossEstimationEnabled) {
                //pathloss exponent estimation enabled
                mEstimatedPathLossExponent = a[pos];

                if (mEstimatedCovariance != null) {
                    mEstimatedPathLossExponentVariance = mEstimatedCovariance.
                            getElementAt(pos, pos);
                }
            } else {
                //pathloss exponent estimation disabled
                mEstimatedPathLossExponent = mInitialPathLossExponent;
                mEstimatedPathLossExponentVariance = null;
            }


            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } catch (final NumericalException e) {
            throw new RadioSourceEstimationException(e);
        } finally {
            mLocked = false;
        }
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
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     *
     * @return estimated path loss exponent.
     */
    public double getEstimatedPathLossExponent() {
        return mEstimatedPathLossExponent;
    }

    /**
     * Gets estimated transmitted power variance.
     * This value will only be available when transmitted power
     * estimation is enabled.
     *
     * @return estimated transmitted power variance or null.
     */
    public Double getEstimatedTransmittedPowerVariance() {
        return mEstimatedTransmittedPowerVariance;
    }

    /**
     * Gets estimated path loss exponent variance.
     * This value will only be available when pathloss
     * exponent estimation is enabled.
     *
     * @return estimated path loss exponent variance or null.
     */
    public Double getEstimatedPathLossExponentVariance() {
        return mEstimatedPathLossExponentVariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return mChiSq;
    }

    /**
     * Setups fitter to estimated position.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterPosition() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent (which is typically 2.0)

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //compute k as the constant part of the isotropic received power formula
        //so that: Pr = Pte*k/d^n
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency),
                mInitialPathLossExponent);
        final double kdB = 10.0 * Math.log10(k);

        final int dims = getNumberOfDimensions();
        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k / d^n) = 10*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return dims;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[dims];
                computeInitialPosition(initial, dims);
                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                double sqrDistance = 0.0;
                double diff;
                for (int j = 0; j < dims; j++) {
                    diff = params[j] - point[j];
                    sqrDistance += diff * diff;

                    //n is mInitialPathLossExponent, which is typically 2.0
                    derivatives[j] = -10.0 * mInitialPathLossExponent * diff;
                }

                //derivatives respect position coordinates are (2D case):
                //f(x,y) = -5*n*log((x - xap)^2 + (y - yap)^2)
                //df/dx = -5*n*2*(x - xap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffX/(ln(10)*sqrDistance)
                //df/dy = -5*n*2*(y - yap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffY/(ln(10)*sqrDistance)
                final double ln10PerSqrDistance = Math.log(10.0) * sqrDistance;
                if (ln10PerSqrDistance != 0.0) {
                    for (int j = 0; j < dims; j++) {
                        derivatives[j] /= ln10PerSqrDistance;
                    }
                }

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //          received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*log(k) + 10*log(Pte) - 5*n*log(d^2) =
                return kdB + initialTransmittedPowerdBm
                        - 5.0 * mInitialPathLossExponent * Math.log10(sqrDistance);
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, dims);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);
                final P position = reading.getPosition();

                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, position.getInhomogeneousCoordinate(j));
                }

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimate transmitted power.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterTransmittedPower() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent (which is typically 2.0)

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //compute k as the constant part of the isotropic received power formula
        //so that: Pr = Pte*k/d^n
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency),
                mInitialPathLossExponent);
        final double kdB = 10.0 * Math.log10(k);
        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k / d^n) = 10*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return 1;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[1];
                initial[0] = initialTransmittedPowerdBm;
                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                final double sqrDistance = mInitialPosition.sqrDistanceTo(mReadings.get(i).getPosition());

                final double transmittedPowerdBm = params[0];

                //derivative respect transmitted power Pt (dBm) = 10*log(Pte)
                derivatives[0] = 1.0;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //          received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*log(k) + 10*log(Pte) - 5*n*log(d^2) =
                return kdB + transmittedPowerdBm
                        - 5.0 * mInitialPathLossExponent * Math.log10(sqrDistance);
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, 1);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];

            x.initialize(initialTransmittedPowerdBm);
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimated path loss exponent.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterPathLossExponent() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //k is defined so that: Pr = Pte * k^n / d^n so that
        //k = (c/(4*pi*f))
        final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);
        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k^n / d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return 1;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[1];
                initial[0] = mInitialPathLossExponent;
                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                final double sqrDistance = mInitialPosition.sqrDistanceTo(mReadings.get(i).getPosition());
                final double pathLossExponent = params[0];

                //derivative respect to path loss exponent
                //f(x,y,n) = n*kdB -5*n*log((x - xap)^2 + (y - yap)^2)
                //df/dn = kdB -5*log((x - xap)^2 + (y - yap)^2) = kdB - 5*log(sqrDistance)
                final double logSqrDistance = Math.log10(sqrDistance);
                derivatives[0] = kdB - 5 * logSqrDistance;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //          received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k^n) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k^n) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*n*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*n*log(k) + 10*log(Pte) - 5*n*log(d^2) =
                return pathLossExponent * kdB + initialTransmittedPowerdBm
                        - 5.0 * pathLossExponent * logSqrDistance;
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, 1);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];

            x.initialize(mInitialPathLossExponent);
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimate transmitted power and position.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterPositionAndTransmittedPower() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent (which is typically 2.0)

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //compute k as the constant part of the isotropic received power formula
        //so that: Pr = Pte*k/d^n
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency),
                mInitialPathLossExponent);
        final double kdB = 10.0 * Math.log10(k);

        final int dims = getNumberOfDimensions();
        final int dimsPlus1 = dims + 1;

        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k / d^n) = 10*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

            @Override
            public int getNumberOfDimensions() {
                return dimsPlus1;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[dimsPlus1];

                //initial position
                computeInitialPosition(initial, dims);

                //initial transmitted power
                initial[dims] = initialTransmittedPowerdBm;

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                double sqrDistance = 0.0;
                double diff;
                for (int j = 0; j < dims; j++) {
                    diff = params[j] - point[j];
                    sqrDistance += diff * diff;

                    //n is mInitialPathLossExponent, which is typically 2.0
                    derivatives[j] = -10.0 * mInitialPathLossExponent * diff;
                }

                final double transmittedPowerdBm = params[dims];

                //derivatives respect position coordinates are (2D case):
                //f(x,y) = -5*n*log((x - xap)^2 + (y - yap)^2)
                //df/dx = -5*n*2*(x - xap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffX/(ln(10)*sqrDistance)
                //df/dy = -5*n*2*(y - yap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffY/(ln(10)*sqrDistance)
                final double ln10PerSqrDistance = Math.log(10.0) * sqrDistance;
                if (ln10PerSqrDistance != 0.0) {
                    for (int j = 0; j < dims; j++) {
                        derivatives[j] /= ln10PerSqrDistance;
                    }
                }

                //derivative respect transmitted power
                derivatives[dims] = 1.0;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //              received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*log(k) + 10*log(Pte) - 5*n*log(d^2) =
                return kdB + transmittedPowerdBm
                        - 5.0 * mInitialPathLossExponent * Math.log10(sqrDistance);
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, dimsPlus1);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);
                final P position = reading.getPosition();

                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, position.getInhomogeneousCoordinate(j));
                }
                x.setElementAt(i, dims, initialTransmittedPowerdBm);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimate position and path loss exponent.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterPositionAndPathLossExponent() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent (which is typically 2.0)

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //compute k as the constant part of the isotropic received power formula
        //so that: Pr = Pte*k/d^n
        final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);

        final int dims = getNumberOfDimensions();
        final int dimsPlus1 = dims + 1;

        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k / d^n) = 10*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return dimsPlus1;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[dimsPlus1];

                //initial position
                computeInitialPosition(initial, dims);

                //initial path loss exponent
                initial[dims] = mInitialPathLossExponent;
                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                double sqrDistance = 0.0;
                double diff;
                final double pathLossExponent = params[dims];
                for (int j = 0; j < dims; j++) {
                    diff = params[j] - point[j];
                    sqrDistance += diff * diff;

                    //n is mInitialPathLossExponent, which is typically 2.0
                    derivatives[j] = -10.0 * pathLossExponent * diff;
                }

                //derivatives respect position coordinates are (2D case):
                //f(x,y,n) = n*kdB -5*n*log((x - xap)^2 + (y - yap)^2)
                //df/dx = -5*n*2*(x - xap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffX/(ln(10)*sqrDistance)
                //df/dy = -5*n*2*(y - yap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffY/(ln(10)*sqrDistance)
                //df/dn = kdB -5*log((x - xap)^2 + (y - yap)^2) = kdB - 5*log(sqrDistance)
                final double ln10PerSqrDistance = Math.log(10.0) * sqrDistance;
                if (ln10PerSqrDistance != 0.0) {
                    for (int j = 0; j < dims; j++) {
                        derivatives[j] /= ln10PerSqrDistance;
                    }
                }

                //derivative respect to path loss exponent
                final double logSqrDistance = Math.log10(sqrDistance);
                derivatives[dims] = kdB - 5 * logSqrDistance;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2
                //k = (c/(4*pi*f))^n

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //              received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k^n) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k^n) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*n*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*n*log(k) + 10*log(Pte) - 5*n*log(d^2) =

                return pathLossExponent * kdB + initialTransmittedPowerdBm
                        - 5.0 * pathLossExponent * logSqrDistance;
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, dimsPlus1);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);
                final P position = reading.getPosition();

                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, position.getInhomogeneousCoordinate(j));
                }
                x.setElementAt(i, dims, mInitialPathLossExponent);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimate transmitted power and path loss exponent.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterTransmittedPowerAndPathLossExponent() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //k is defined so that: Pr = Pte * k^n / d^n so that
        //k = (c/(4*pi*f))
        final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);

        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k^n / d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return 2;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[2];

                //initial transmitted power
                initial[0] = initialTransmittedPowerdBm;

                //initial path loss exponent
                initial[1] = mInitialPathLossExponent;

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                final double sqrDistance = mInitialPosition.sqrDistanceTo(mReadings.get(i).getPosition());
                final double transmittedPowerdBm = params[0];
                final double pathLossExponent = params[1];

                //derivative respect transmitted power
                derivatives[0] = 1.0;

                //derivative respect to path loss exponent
                final double logSqrDistance = Math.log10(sqrDistance);
                derivatives[1] = kdB - 5 * logSqrDistance;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2
                //k = (c/(4*pi*f))^n

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //                 received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k^n) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k^n) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*n*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*n*log(k) + 10*log(Pte) - 5*n*log(d^2) =

                return pathLossExponent * kdB + transmittedPowerdBm
                        - 5.0 * pathLossExponent * logSqrDistance;
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, 2);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);

                x.setElementAt(i, 0, initialTransmittedPowerdBm);
                x.setElementAt(i, 1, mInitialPathLossExponent);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Setups fitter to estimate transmitted power, position and path
     * loss exponent.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    private void setupFitterPositionTransmittedPowerAndPathLossExponent() throws FittingException {
        //because all readings must belong to the same radio source, we
        //obtain the frequency of the first radio source on the first reading
        RssiReadingLocated<S, P> reading = mReadings.get(0);
        final double frequency = reading.getSource().getFrequency();

        //n = 2.0, is the path loss exponent

        //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^n/((4*pi*f)^n * d^n)


        //k is defined so that: Pr = Pte * k^n / d^n so that
        //k = (c/(4*pi*f))
        final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);

        final int dims = getNumberOfDimensions();
        final int dimsPlus1 = dims + 1;
        final int dimsPlus2 = dims + 2;

        final double initialTransmittedPowerdBm = computeInitialTransmittedPowerdBm();

        //for numerical accuracy reasons, a logarithmic version of the previous
        //formula will be used instead
        //Pr (dBm) = 10 * log(Pte * k^n / d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d)

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

            @Override
            public int getNumberOfDimensions() {
                return dimsPlus2;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[dimsPlus2];

                //initial position
                computeInitialPosition(initial, dims);

                //initial transmitted power
                initial[dims] = initialTransmittedPowerdBm;

                //initial path loss exponent
                initial[dimsPlus1] = mInitialPathLossExponent;

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params,
                    final double[] derivatives) {
                double sqrDistance = 0.0;
                double diff;
                final double pathLossExponent = params[dimsPlus1];
                for (int j = 0; j < dims; j++) {
                    diff = params[j] - point[j];
                    sqrDistance += diff * diff;

                    //n is mInitialPathLossExponent, which is typically 2.0
                    derivatives[j] = -10.0 * pathLossExponent * diff;
                }

                final double transmittedPowerdBm = params[dims];

                //derivatives respect position coordinates are (2D case):
                //f(x,y,n) = n*kdB -5*n*log((x - xap)^2 + (y - yap)^2)
                //df/dx = -5*n*2*(x - xap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffX/(ln(10)*sqrDistance)
                //df/dy = -5*n*2*(y - yap)/(ln(10)*((x - xap)^2 + (y - yap)^2)) = -10*n*diffY/(ln(10)*sqrDistance)
                //df/dn = kdB -5*log((x - xap)^2 + (y - yap)^2) = kdB - 5*log(sqrDistance)
                final double ln10PerSqrDistance = Math.log(10.0) * sqrDistance;
                if (ln10PerSqrDistance != 0.0) {
                    for (int j = 0; j < dims; j++) {
                        derivatives[j] /= ln10PerSqrDistance;
                    }
                }

                //derivative respect transmitted power
                derivatives[dims] = 1.0;

                //derivative respect to path loss exponent
                final double logSqrDistance = Math.log10(sqrDistance);
                derivatives[dimsPlus1] = kdB - 5 * logSqrDistance;

                //d^2 = (x - xap)^2 + (y - yap)^2
                //d^n = (d^2)^n/2
                //k = (c/(4*pi*f))^n

                //Pr = Pt*Gt*Gr*lambda^n/(4*pi*d)^n,    where Pr is the received power
                //n is the path loss exponent
                //lambda = c/f, where lambda is wavelength,
                //Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the
                //          received Gain
                //Pr = Pte*c^n/((4*pi*f)^n * d^n)
                //Pr (dBm) = 10*log(k^n) + 10*log(Pte) - 10*log(d^n) =
                //10*log(k^n) + 10*log(Pte) - 10*log((d^2)^n/2) =
                //10*n*log(k) + 10*log(Pte) - 10*n/2*log(d^2) =
                //10*n*log(k) + 10*log(Pte) - 5*n*log(d^2) =

                return pathLossExponent * kdB + transmittedPowerdBm
                        - 5.0 * pathLossExponent * logSqrDistance;
            }
        });

        final int numReadings = mReadings.size();
        try {
            final Matrix x = new Matrix(numReadings, dimsPlus2);
            final double[] y = new double[numReadings];
            final double[] standardDeviations = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                reading = mReadings.get(i);
                final P position = reading.getPosition();

                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, j, position.getInhomogeneousCoordinate(j));
                }
                x.setElementAt(i, dims, initialTransmittedPowerdBm);
                x.setElementAt(i, dimsPlus1, mInitialPathLossExponent);

                standardDeviations[i] = reading.getRssiStandardDeviation() != null ?
                        reading.getRssiStandardDeviation() :
                        DEFAULT_POWER_STANDARD_DEVIATION;
                y[i] = reading.getRssi();
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException ignore) {
            //never happens
        }
    }

    /**
     * Computes initial transmitted power expressed in dBm's.
     * If no initial transmitted power is provided, the average of all measures
     * is used, otherwise provided value is used.
     *
     * @return initial transmitted power.
     */
    private double computeInitialTransmittedPowerdBm() {
        if (mInitialTransmittedPowerdBm == null) {
            //compute average transmitted power (in mW)
            final int num = mReadings.size();
            double result = 0.0;
            for (final RssiReadingLocated<S, P> reading : mReadings) {
                final double rssi = reading.getRssi();
                result += rssi / (double) num;
            }
            return result;
        } else {
            //convert initial value
            return mInitialTransmittedPowerdBm;
        }
    }

    /**
     * Computes initial position.
     *
     * @param result array where result will be stored.
     * @param dims   number of dimensions of position coordinates (either 2 or 3).
     */
    private void computeInitialPosition(final double[] result, final int dims) {
        if (mInitialPosition == null) {
            final int num = mReadings.size();

            //compute average centroid of fingerprint positions
            for (final RssiReadingLocated<S, P> reading : mReadings) {
                final P position = reading.getPosition();
                for (int i = 0; i < dims; i++) {
                    result[i] += position.getInhomogeneousCoordinate(i) /
                            (double) num;
                }
            }
        } else {
            //copy initial position
            for (int i = 0; i < dims; i++) {
                result[i] = mInitialPosition.getInhomogeneousCoordinate(i);
            }
        }
    }
}
