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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;

import java.util.List;

/**
 * Estimates 3D position and transmitted power of a WiFi access point assuming that the
 * access point emits isotropically following the expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antena
 * Gr is the Gain of the receiver antena
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the WiFi signal.
 * Because usually information about the antena of the Wifi Access Point cannot be
 * retrieved (because many measurements are made on unkown access points where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 */
@SuppressWarnings("WeakerAccess")
public class WifiAccessPointPowerAndPositionEstimator3D extends
        WifiAccessPointPowerAndPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public WifiAccessPointPowerAndPositionEstimator3D() { }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition)
            throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
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
    public WifiAccessPointPowerAndPositionEstimator3D(
            Double initialTransmittedPowerdBm,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     */
    public WifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's).
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                estimation of access point transmitted power
     *                                (expressed in dBm's)
     * @param initialPathLossExponent initial path loss exponent. A typical value is 2.0.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public WifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            double initialPathLossExponent,
            WifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
    }

    /**
     * Gets minimum required number of readings to estimate power
     * and position.
     * This is always 4 readings.
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 3.
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated access point 3D position.
     * @return estimated access point 3D position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Gets estimated located access point with estimated transmitted power.
     * @return estimasted located access point with estimated transmitted power.
     */
    @Override
    @SuppressWarnings("unchecked")
    public WifiAccessPointWithPowerAndLocated3D getEstimatedAccessPoint() {
        List<? extends WifiRssiReadingLocated<Point3D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        WifiAccessPoint accessPoint = readings.get(0).getAccessPoint();

        Point3D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();
        if (estimatedPositionCovariance == null) {
            //covariance not available
            return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(),
                    accessPoint.getFrequency(), accessPoint.getSsid(),
                    getEstimatedTransmittedPowerdBm(), estimatedPosition);
        } else {
            //covariance is available
            return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(),
                    accessPoint.getFrequency(), accessPoint.getSsid(),
                    getEstimatedTransmittedPowerdBm(),
                    Math.sqrt(getEstimatedTransmittedPowerVariance()),
                    estimatedPosition,
                    estimatedPositionCovariance);
        }
    }
}
