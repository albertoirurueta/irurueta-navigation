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

/**
 * Data related to a WiFi access point with estimated transmitted power and
 * known 2D location.
 */
public class WifiAccessPointWithPowerAndLocated2D extends
        WifiAccessPointWithPowerAndLocated<Point2D> {

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param position position where access point is located.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Point2D position) {
        super(bssid, frequency, transmittedPower, position);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency, String ssid,
            double transmittedPower, Point2D position) {
        super(bssid, frequency, ssid, transmittedPower, position);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            Point2D position) {
        super(bssid, frequency, transmittedPower,
                transmittedPowerStandardDeviation, position);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            Point2D position) {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, position);
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
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Point2D position, Matrix positionCovariance) {
        super(bssid, frequency, transmittedPower, position, positionCovariance);
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
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency, String ssid,
            double transmittedPower, Point2D position, Matrix positionCovariance) {
        super(bssid, frequency, ssid, transmittedPower, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            Point2D position, Matrix positionCovariance) {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                position, positionCovariance);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            String ssid, double transmittedPower,
            Double transmittedPowerStandardDeviation, Point2D position,
            Matrix positionCovariance) {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, position, positionCovariance);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, double pathLossExponent, Point2D position) {
        super(bssid, frequency, transmittedPower, pathLossExponent, position);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Point2D position) {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, position);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            String ssid, double transmittedPower,
            Double transmittedPowerStandardDeviation, double pathLossExponent,
            Point2D position) {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent,
                position);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, double pathLossExponent, Point2D position,
            Matrix positionCovariance) {
        super(bssid, frequency, transmittedPower, pathLossExponent, position,
                positionCovariance);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Point2D position, Matrix positionCovariance) {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            String ssid, double transmittedPower,
            Double transmittedPowerStandardDeviation, double pathLossExponent,
            Point2D position, Matrix positionCovariance) {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent, position,
                positionCovariance);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            Point2D position) {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, pathLossExponentStandardDeviation, position);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            Point2D position) {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            Point2D position, Matrix positionCovariance)
            throws IllegalArgumentException {
        super(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, pathLossExponentStandardDeviation, position,
                positionCovariance);
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
    public WifiAccessPointWithPowerAndLocated2D(String bssid, double frequency, String ssid,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            Point2D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(bssid, frequency, ssid, transmittedPower,
                transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointWithPowerAndLocated2D() {
        super();
    }
}
