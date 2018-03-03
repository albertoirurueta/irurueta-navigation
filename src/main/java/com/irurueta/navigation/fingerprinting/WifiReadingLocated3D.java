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
import com.irurueta.geometry.Point3D;

/**
 * Contains a 3D located signal strength reading associated to a given WiFi
 * access point.
 */
@SuppressWarnings("WeakerAccess")
public class WifiReadingLocated3D extends
        WifiReadingLocated<Point3D> {

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if either access point data or position
     * are null.
     */
    public WifiReadingLocated3D(WifiAccessPoint accessPoint, double rssi,
            Point3D position) throws IllegalArgumentException {
        super(accessPoint, rssi, position);
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
    public WifiReadingLocated3D(WifiAccessPoint accessPoint, double rssi,
            Point3D position, Double rssiStandardDeviation)
            throws IllegalArgumentException {
        super(accessPoint, rssi, position, rssiStandardDeviation);
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
    public WifiReadingLocated3D(WifiAccessPoint accessPoint, double rssi,
            Point3D position, Matrix positionCovariance)
            throws IllegalArgumentException {
        super(accessPoint, rssi, position, positionCovariance);
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
    public WifiReadingLocated3D(WifiAccessPoint accessPoint, double rssi,
            Point3D position, Double rssiStandardDeviation,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(accessPoint, rssi, position, rssiStandardDeviation,
                positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected WifiReadingLocated3D() {
        super();
    }

}
