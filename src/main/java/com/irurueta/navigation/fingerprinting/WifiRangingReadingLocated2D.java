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
import com.irurueta.geometry.Point2D;

/**
 * Contains a located ranging reading associated to a given WiFi access point,
 * indicating the distance to such access point.
 */
@SuppressWarnings("WeakerAccess")
public class WifiRangingReadingLocated2D extends WifiRangingReadingLocated<Point2D> {

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if access point data is null, distance is negative
     * or position is null.
     */
    public WifiRangingReadingLocated2D(WifiAccessPoint accessPoint, double distance,
            Point2D position) throws IllegalArgumentException {
        super(accessPoint, distance, position);
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if access point data is null, distance is negative,
     * position is null or standard deviation is zero or negative.
     */
    public WifiRangingReadingLocated2D(WifiAccessPoint accessPoint, double distance,
            Point2D position, Double distanceStandardDeviation)
            throws IllegalArgumentException {
        super(accessPoint, distance, position, distanceStandardDeviation);
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if access point data is null, distance is negative
     * or position is null.
     */
    public WifiRangingReadingLocated2D(WifiAccessPoint accessPoint, double distance,
            Point2D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(accessPoint, distance, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if access point data is null, distance is negative,
     * position is null or standard deviation is zero or negative.
     */
    public WifiRangingReadingLocated2D(WifiAccessPoint accessPoint, double distance,
            Point2D position, Double distanceStandardDeviation,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(accessPoint, distance, position, distanceStandardDeviation,
                positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected WifiRangingReadingLocated2D() {
        super();
    }
}
