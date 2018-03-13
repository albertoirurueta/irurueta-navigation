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

/**
 * Contains a ranging reading associated to a given WiFi
 * access point, indicating the distance to such access point.
 * @param <AP> a {@link WifiAccessPoint} type.
 */
@SuppressWarnings("WeakerAccess")
public class WifiRangingReading<AP extends WifiAccessPoint> extends WifiReading<AP> {

    /**
     * Distance in meters to the access point.
     */
    private double mDistance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double mDistanceStandardDeviation;

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @throws IllegalArgumentException if access point data is null or distance is negative.
     */
    public WifiRangingReading(AP accessPoint, double distance)
            throws IllegalArgumentException {
        super(accessPoint);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        mDistance = distance;
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if access point data is null, distance is negative or
     * standard deviation is zero or negative.
     */
    public WifiRangingReading(AP accessPoint, double distance,
                              Double distanceStandardDeviation) throws IllegalArgumentException {
        this(accessPoint, distance);

        if (distanceStandardDeviation != null && distanceStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }

        mDistanceStandardDeviation = distanceStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected WifiRangingReading() {
        super();
    }

    /**
     * Gets distance in meters to the access point.
     * @return distance in mters to the access point.
     */
    public double getDistance() {
        return mDistance;
    }

    /**
     * Gets standard deviation of distance, if available.
     * @return standard deviation of distance or null.
     */
    public Double getDistanceStandardDeviation() {
        return mDistanceStandardDeviation;
    }
}
