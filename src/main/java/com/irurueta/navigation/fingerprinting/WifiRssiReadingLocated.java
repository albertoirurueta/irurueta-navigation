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
 * Contains a located signal strength reading associated to a given WiFi
 * access point.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public class WifiRssiReadingLocated<P extends Point> extends WifiRssiReading {

    /**
     * Position where WiFi reading was made.
     */
    private P mPosition;

    /**
     * Covariance of inhomogeneous coordinates of current position
     * (if available).
     */
    private Matrix mPositionCovariance;

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if either access point data or position
     * are null.
     */
    public WifiRssiReadingLocated(WifiAccessPoint accessPoint, double rssi, P position)
            throws IllegalArgumentException {
        super(accessPoint, rssi);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if either access point data or position
     * are null.
     */
    public WifiRssiReadingLocated(WifiAccessPoint accessPoint, double rssi,
                                  P position, Double rssiStandardDeviation)
            throws IllegalArgumentException {
        super(accessPoint, rssi, rssiStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either access point or position are
     * null, or covariance has invalid size.
     */
    public WifiRssiReadingLocated(WifiAccessPoint accessPoint, double rssi,
                                  P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(accessPoint, rssi, position);

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
     * @param accessPoint access point associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either access point or position are
     * null, or covariance has invalid size.
     */
    public WifiRssiReadingLocated(WifiAccessPoint accessPoint, double rssi,
            P position, Double rssiStandardDeviation, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(accessPoint, rssi, position, rssiStandardDeviation);

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
    protected WifiRssiReadingLocated() {
        super();
    }

    /**
     * Gets position where reading was made.
     * @return position where reading was made.
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
