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
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Estimates 3D position, transmitted power and path loss exponent of a
 * radio source (e.g. WiFi access point or bluetooth beacon) assuming
 * that the ranging data is available to obtain position with greater
 * accuracy and that the radio source emits isotropically following the
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
 * This implementation is like RangingAndRssiRadioSourceEstimator3D but allows mixing
 * different kinds of located radio source readings (ranging, RSSI and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class MixedRadioSourceEstimator3D<S extends RadioSource>
        extends MixedRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public MixedRadioSourceEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same
     *                 radio sources.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator3D(
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public MixedRadioSourceEstimator3D(Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator3D(Point3D initialPosition,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerDbm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public MixedRadioSourceEstimator3D(Double initialTransmittedPowerDbm) {
        super(initialTransmittedPowerDbm);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator3D(Double initialTransmittedPowerdBm,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm) {
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
    public MixedRadioSourceEstimator3D(Point3D initialPosition,
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
    public MixedRadioSourceEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        super(readings, initialPosition,initialTransmittedPowerdBm,
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
    public MixedRadioSourceEstimator3D(Point3D initialPosition,
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
    public MixedRadioSourceEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm, double initialPathLossExponent,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator3D(
            List<? extends ReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            MixedRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Gets number of dimensions of position points.
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source.
     * @return estimated located radio source or null.
     */
    @SuppressWarnings("unchecked")
    @Override
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

        Double transmittedPowerVariance = getEstimatedTransmittedPowerVariance();
        Double transmittedPowerStandardDeviation = transmittedPowerVariance != null ?
                Math.sqrt(transmittedPowerVariance) : null;

        double pathLossExponent = getEstimatedPathLossExponent();

        Double pathLossExponentVariance = getEstimatedPathLossExponentVariance();
        Double pathLossExponentStandardDeviation = pathLossExponentVariance != null ?
                Math.sqrt(pathLossExponentVariance) : null;

        if (source instanceof WifiAccessPoint) {
            WifiAccessPoint accessPoint = (WifiAccessPoint)source;
            if (transmittedPowerdBm != null) {
                return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(),
                        accessPoint.getFrequency(), accessPoint.getSsid(),
                        transmittedPowerdBm,
                        transmittedPowerStandardDeviation,
                        pathLossExponent, pathLossExponentStandardDeviation,
                        estimatedPosition, estimatedPositionCovariance);
            } else {
                return new WifiAccessPointLocated3D(accessPoint.getBssid(),
                        accessPoint.getFrequency(), accessPoint.getSsid(),
                        estimatedPosition, estimatedPositionCovariance);
            }
        } else if (source instanceof Beacon) {
            Beacon beacon = (Beacon)source;
            //transmitted power does not need to be estimated for beacons because
            //they broadcast such information
            return new BeaconWithPowerAndLocated3D(beacon.getIdentifiers(),
                    beacon.getTransmittedPower(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(), pathLossExponent,
                    transmittedPowerStandardDeviation,
                    pathLossExponentStandardDeviation, estimatedPosition,
                    estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Creates inner estimators if needed.
     */
    @Override
    protected void createInnerEstimatorsIfNeeded() {
        if (mRangingInnerEstimator == null) {
            mRangingInnerEstimator = new RangingRadioSourceEstimator3D<>();
        }

        if (mRssiInnerEstimator == null &&
                (mTransmittedPowerEstimationEnabled || mPathLossEstimationEnabled)) {
            mRssiInnerEstimator = new RssiRadioSourceEstimator3D<>();
        }
    }
}
