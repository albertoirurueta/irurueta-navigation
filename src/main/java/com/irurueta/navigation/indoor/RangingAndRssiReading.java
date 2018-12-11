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

/**
 * Contains a reading associated to a given radio source (e.e. WiFi access point or
 * bluetooth beacon) containing signal strength and distance to associated access
 * point.
 * @param <S> a {@link RadioSource} type.
 */
public class RangingAndRssiReading<S extends RadioSource> extends Reading<S> {

    /**
     * Distance in meters to the radio source.
     */
    private double mDistance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double mDistanceStandardDeviation;

    /**
     * Received signal strength indicator (of this 802.11 network for a WiFi access point or
     * of this bluetooth beacon), in dBm.
     */
    private double mRssi;

    /**
     * Standard deviation of RSSI, if available.
     */
    private Double mRssiStandardDeviation;

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param rssi received signal strength indicator in dBm.
     * @throws IllegalArgumentException if radio source data is null or distance is negative.
     */
    public RangingAndRssiReading(S source, double distance, double rssi) {
        super(source);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        mDistance = distance;
        mRssi = rssi;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param rssi received signal strength indicator in dBm.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     * or any of the standard deviations is zero or negative.
     */
    public RangingAndRssiReading(S source, double distance, double rssi,
            Double distanceStandardDeviation, Double rssiStandardDeviation) {
        this(source, distance, rssi);

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
    protected RangingAndRssiReading() {
        super();
    }

    /**
     * Contains radio source reading type.
     * @return reading type.
     */
    @Override
    public ReadingType getType() {
        return ReadingType.RANGING_AND_RSSI_READING;
    }

    /**
     * Gets distance in meters to the radio source.
     * @return distance in mters to the radio source.
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
     * Gets received signal strength indicator of (of this 802.11 network for a WiFi access point or
     * of this bluetooth beacon), in dBm.
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
