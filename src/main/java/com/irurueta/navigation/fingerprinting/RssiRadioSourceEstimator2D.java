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
package com.irurueta.navigation.fingerprinting;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;

import java.util.List;

/**
 * Estimates 2D position, transmitted power and pathloss exponent
 * of a radio source assuming that the radio source emits
 * isotropically following the expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antena
 * Gr is the Gain of the receiver antena
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * Because usually information about the antena of the radio source cannot be
 * retrieved (because many measurements are made on unkown access points where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 *
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
 */
@SuppressWarnings("WeakerAccess")
public class RssiRadioSourceEstimator2D<S extends RadioSource> extends
        RssiRadioSourceEstimator<S, Point2D> {

    /**
     * Constructor.
     */
    public RssiRadioSourceEstimator2D() { }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator2D(
            RssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            RssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     */
    public RssiRadioSourceEstimator2D(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition)
            throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator2D(Point2D initialPosition,
                                      RssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     */
    public RssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator2D(
            Double initialTransmittedPowerdBm,
            RssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Double initialTransmittedPowerdBm,
            RssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     */
    public RssiRadioSourceEstimator2D(Point2D initialPosition,
                                      Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator2D(Point2D initialPosition,
                                      Double initialTransmittedPowerdBm,
                                      RssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            RssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
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
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     */
    public RssiRadioSourceEstimator2D(
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
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator2D(
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of radio source transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator2D(
            List<? extends RssiReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            RssiRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Gets minimum required number of readings to estimate power
     * and position.
     * This is always 3 readings.
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 2.
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated radio source 2D position.
     * @return estimated radio source 2D position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint2D result = new InhomogeneousPoint2D();
        getEstimatedPosition(result);
        return result;

    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceWithPowerAndLocated<Point2D> getEstimatedRadioSource() {
        List<? extends RssiReadingLocated<S, Point2D>> readings = getReadings();
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
}
