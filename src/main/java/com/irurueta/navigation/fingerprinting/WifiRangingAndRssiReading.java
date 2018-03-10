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
 * Contains a reading associated to a given WiFi access point containing
 * signal strength and distance to associated access point.
 */
@SuppressWarnings("WeakerAccess")
public class WifiRangingAndRssiReading extends WifiReading {

    /**
     * Distance in meters to the access point.
     */
    private double mDistance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double mDistanceStandardDeviation;

    /**
     * Received signal strength indicator of this 802.11 network, in dBm.
     */
    private double mRssi;

    /**
     * Standard deviation of RSSI, if available.
     */
    private Double mRssiStandardDeviation;

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param rssi received signal strength indicator in dBm.
     * @throws IllegalArgumentException if access point data is null or distance is negative.
     */
    public WifiRangingAndRssiReading(WifiAccessPoint accessPoint, double distance, double rssi)
        throws IllegalArgumentException {
        super(accessPoint);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        mDistance = distance;
        mRssi = rssi;
    }

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @param distance distance in meters to the access point.
     * @param rssi received signal strength indicator in dBm.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if access point data is null, distance is negative
     * or any of the standard deviations is zero or negative.
     */
    public WifiRangingAndRssiReading(WifiAccessPoint accessPoint, double distance, double rssi,
            Double distanceStandardDeviation, Double rssiStandardDeviation)
            throws IllegalArgumentException {
        this(accessPoint, distance, rssi);

        if (distanceStandardDeviation != null && distanceStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }
        if (rssiStandardDeviation != null && rssiStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }

        mDistanceStandardDeviation = distanceStandardDeviation;
        mRssiStandardDeviation = rssiStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected WifiRangingAndRssiReading() {
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

    /**
     * Gets received signal strength indicator of this 802.11 network, in dBm.
     * @return received signal strength indicator.
     */
    public double getRssi() {
        return mRssi;
    }

    /**
     * Gets standard deviation of RSSI, if available.
     * @return standard deviation of RSSI, if available.
     */
    public Double getRssiStandardDeviation() {
        return mRssiStandardDeviation;
    }
}
