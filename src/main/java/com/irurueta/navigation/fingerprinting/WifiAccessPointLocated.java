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
 * Data related to a WiFi access point whose location is known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class WifiAccessPointLocated<P extends Point>
        extends WifiAccessPoint implements RadioSourceLocated<P> {

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
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     * frequency is negative.
     */
    public WifiAccessPointLocated(String bssid, double frequency, P position)
            throws IllegalArgumentException {
        super(bssid, frequency);

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
     * @param position position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     * frequency is negative.
     */
    public WifiAccessPointLocated(String bssid, double frequency, String ssid,
            P position) throws IllegalArgumentException {
        super(bssid, frequency, ssid);

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
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID or position are null or
     * frequency is negative, or covariance does not have proper size.
     */
    public WifiAccessPointLocated(String bssid, double frequency, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(bssid, frequency, position);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param position position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID or position are null or
     * frequency is negative, or covariance does not have proper size.
     */
    public WifiAccessPointLocated(String bssid, double frequency, String ssid,
            P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(bssid, frequency, ssid, position);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointLocated() {
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
