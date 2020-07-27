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
 * Data related to a WiFi access point whose 2D location is known.
 */
public class WifiAccessPointLocated2D extends WifiAccessPointLocated<Point2D> {

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param position  position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    public WifiAccessPointLocated2D(
            final String bssid, final double frequency, final Point2D position) {
        super(bssid, frequency, position);
    }

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid      service set identifier (SSID) of this 802.11 network.
     * @param position  position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    public WifiAccessPointLocated2D(
            final String bssid, final double frequency, final String ssid,
            final Point2D position) {
        super(bssid, frequency, ssid, position);
    }

    /**
     * Constructor.
     *
     * @param bssid              basic service set identifier of this access point in the form of a six-byte MAC address:
     *                           xx:xx:xx:xx:xx:xx.
     * @param frequency          frequency used by this Access Point (expressed in Hz).
     * @param position           position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    public WifiAccessPointLocated2D(
            final String bssid, final double frequency, final Point2D position,
            final Matrix positionCovariance) {
        super(bssid, frequency, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param bssid              basic service set identifier of this access point in the form of a six-byte MAC address:
     *                           xx:xx:xx:xx:xx:xx.
     * @param frequency          frequency used by this Access Point (expressed in Hz).
     * @param ssid               service set identifier (SSID) of this 802.11 network.
     * @param position           position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).*
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    public WifiAccessPointLocated2D(
            final String bssid, final double frequency, final String ssid,
            final Point2D position, final Matrix positionCovariance) {
        super(bssid, frequency, ssid, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointLocated2D() {
        super();
    }
}
