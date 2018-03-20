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
import com.irurueta.geometry.Point;

/**
 * Data related to a WiFi access point with estimated transmitted power and
 * known location.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public class WifiAccessPointWithPowerAndLocated<P extends Point>
        extends WifiAccessPointWithPower implements RadioSourceWithPowerAndLocated<P> {

    /**
     * Position where access point is located.
     */
    private P mPosition;

    /**
     * Covariance of inhomogeneous coordinates of current position (if available).
     */
    private Matrix mPositionCovariance;

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param position position where access point is located.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, P position) throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param position position where access point is located.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, P position) throws IllegalArgumentException {
        super(bssid, frequency, ssid, transmittedPower);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position) throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower,
                transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position) throws IllegalArgumentException {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if BSSID is null, frequency is negative or
     * covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if BSSID is null, frequency is negative
     * or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, ssid, transmittedPower, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * transmitted power standard deviation is negative or covariance is invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * transmitted power standard deviation is negative or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null or frequency is negative
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, double pathLossExponent, P position)
            throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position) throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position) throws IllegalArgumentException {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, double pathLossExponent, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower, pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * transmitted power standard deviation is negative or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * transmitted power standard deviation is negative or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or any standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            P position) throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or any standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            P position) throws IllegalArgumentException {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * any standard deviation is negative or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, pathLossExponentStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * any standard deviation is negative or covariance has invalid size.
     */
    public WifiAccessPointWithPowerAndLocated(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointWithPowerAndLocated() {
        super();
    }

    /**
     * Gets position where access point is located.
     * @return position where access point is located.
     */
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     * @return covariance of position or null.
     */
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
