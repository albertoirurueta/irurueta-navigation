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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 3D position and transmitted power
 * of a WiFi access point, by discarding outliers and assuming that the access
 * point emits isotropically following the expression below:
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
 * If WifiReadings contain RSSI standard deviations, those values will be used,
 * otherwise it will be asumed an RSSI standard deviation of 1 dB.
 * Implementations of this class should be able to detect and discard outliers in
 * order to find the best solution.
 */
@SuppressWarnings("WeakerAccess")
public abstract class RobustWifiAccessPointPowerAndPositionEstimator3D extends
        RobustWifiAccessPointPowerAndPositionEstimator<Point3D> {

    /**
     * Power and 3D position estimator used internally.
     */
    protected WifiAccessPointPowerAndPositionEstimator3D mInnerEstimator =
            new WifiAccessPointPowerAndPositionEstimator3D();

    /**
     * Subset of readings used by inner estimator.
     */
    private List<WifiRssiReadingLocated<Point3D>> mInnerReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings bleonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if fingerprints are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @throws IllegalArgumentException if fingerprints are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition)
            throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if fingerprints are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
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
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener in charge of attending events raised by this instance.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(Point3D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets WiFi signal readings belonging to the same access point.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustWifiAccessPointPowerAndPositionEstimator3D(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D();
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D();
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D();
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D();
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D();
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D();
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D();
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D();
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition);
        }
    }

    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method)
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            Double initialTransmittedPowerdBm, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, initialTransmittedPowerdBm);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, initialTransmittedPowerdBm);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, initialTransmittedPowerdBm,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, initialPosition, initialTransmittedPowerdBm,
                        listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener,
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case LMedS:
                return new LMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case MSAC:
                return new MSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        readings, initialPosition, initialTransmittedPowerdBm, listener);
            case PROSAC:
                return new PROSACRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D(
                        qualityScores, readings, initialPosition,
                        initialTransmittedPowerdBm, listener);
        }
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings)
            throws IllegalArgumentException {
        return create(readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
        return create(readings, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition) {
        return create(initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(readings, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Double initialTransmittedPowerdBm) {
        return create(initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        return create(readings, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(readings, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition, Double initialTransmittedPowerdBm) {
        return create(initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(initialPosition, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(readings, initialPosition, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings containing belonging to
     *                 the same access point.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings)
            throws IllegalArgumentException {
        return create(qualityScores, readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition) {
        return create(qualityScores, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(qualityScores, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param listener listener in charge of attending events raised by this instance
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm) throws IllegalArgumentException {
        return create(qualityScores, readings, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(qualityScores, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores, Point3D initialPosition,
            Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D wifi access point power and position estimator using
     * default method.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings WiFi signal readings belonging to the same access point.
     * @param initialPosition initial position to start the estimation of access
     *                        point position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param listener listener in charge of attending events raised by this instance.
     * @return a new robust 3D wifi access point power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static RobustWifiAccessPointPowerAndPositionEstimator3D create(
            double[] qualityScores,
            List<? extends WifiRssiReadingLocated<Point3D>> readings,
            Point3D initialPosition, Double initialTransmittedPowerdBm,
            RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> listener)
            throws IllegalArgumentException {
        return create(qualityScores, readings, initialPosition,
                initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Gets minimum required number of fingerprint readings to estimate
     * power and position.
     * This is 3 readings for 3D.
     * @return always returns 3 readings.
     */
    @Override
    public int getMinReadings() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
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
     * Solves preliminar solution for a subset of samples.
     * @param samplesIndices indices of subset samples.
     * @param solutions instance where solution will be stored.
     */
    protected void solvePreliminarSolutions(int[] samplesIndices,
                                            List<Solution<Point3D>> solutions) {

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

            mInnerEstimator.setReadings(mInnerReadings);

            mInnerEstimator.estimate();

            Point3D estimatedPosition = mInnerEstimator.getEstimatedPosition();
            double estimatedTransmittedPowerdBm =
                    mInnerEstimator.getEstimatedTransmittedPowerdBm();
            solutions.add(new Solution<>(estimatedPosition,
                    estimatedTransmittedPowerdBm));
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
     */
    protected void attemptRefine(Solution<Point3D> result) {
        Point3D initialPosition = result.getEstimatedPosition();
        double initialTransmittedPowerdBm =
                result.getEstimatedTransmittedPowerdBm();

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
                mInnerEstimator.setReadings(mInnerReadings);

                mInnerEstimator.estimate();

                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = mInnerEstimator.getEstimatedCovariance();
                } else {
                    mCovariance = null;
                }

                mEstimatedPosition = mInnerEstimator.getEstimatedPosition();
                mEstimatedTransmittedPowerdBm =
                        mInnerEstimator.getEstimatedTransmittedPowerdBm();
            } catch (Exception e) {
                //refinement failed, so we return input value
                mCovariance = null;
                mEstimatedPosition = initialPosition;
                mEstimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
            }
        } else {
            mCovariance = null;
            mEstimatedPosition = initialPosition;
            mEstimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
        }
    }
}
