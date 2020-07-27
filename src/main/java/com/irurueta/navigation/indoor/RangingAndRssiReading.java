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
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("WeakerAccess")
public class RangingAndRssiReading<S extends RadioSource> extends Reading<S> {

    /**
     * Default number of measurements.
     */
    public static final int DEFAULT_NUM_MEASUREMENTS = 1;

    /**
     * Distance in meters to the radio source.
     */
    private double mDistance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double mDistanceStandardDeviation;

    /**
     * Number of attempted measurements using in the RTT exchange.
     */
    private int mNumAttemptedMeasurements = DEFAULT_NUM_MEASUREMENTS;

    /**
     * Number of successful measurements used to calculate the distance and standard
     * deviation.
     */
    private int mNumSuccessfulMeasurements = DEFAULT_NUM_MEASUREMENTS;

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
     *
     * @param source   radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param rssi     received signal strength indicator in dBm.
     * @throws IllegalArgumentException if radio source data is null or distance is negative.
     */
    public RangingAndRssiReading(
            final S source, final double distance, final double rssi) {
        super(source);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        mDistance = distance;
        mRssi = rssi;
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  number of attempted measures is less than 1 or number of successful measures is
     *                                  negative.
     */
    public RangingAndRssiReading(
            final S source, final double distance, final double rssi,
            final int numAttemptedMeasurements, final int numSuccessfulMeasurements) {
        this(source, distance, rssi, null, null,
                numAttemptedMeasurements, numSuccessfulMeasurements);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     *                                  or any of the standard deviations is zero or negative.
     */
    public RangingAndRssiReading(
            final S source,
            final double distance,
            final double rssi,
            final Double distanceStandardDeviation,
            final Double rssiStandardDeviation) {
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
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI, if available.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  any of the standard deviations is zero or negative, number of attempted
     *                                  measures is less than 1 or number of successful measures is negative.
     */
    public RangingAndRssiReading(
            final S source,
            final double distance,
            final double rssi,
            final Double distanceStandardDeviation,
            final Double rssiStandardDeviation,
            final int numAttemptedMeasurements,
            final int numSuccessfulMeasurements) {
        this(source, distance, rssi, distanceStandardDeviation, rssiStandardDeviation);

        if (numAttemptedMeasurements < DEFAULT_NUM_MEASUREMENTS ||
                numSuccessfulMeasurements < 0) {
            throw new IllegalArgumentException();
        }

        mNumAttemptedMeasurements = numAttemptedMeasurements;
        mNumSuccessfulMeasurements = numSuccessfulMeasurements;
    }

    /**
     * Empty constructor.
     */
    protected RangingAndRssiReading() {
        super();
    }

    /**
     * Contains radio source reading type.
     *
     * @return reading type.
     */
    @Override
    public ReadingType getType() {
        return ReadingType.RANGING_AND_RSSI_READING;
    }

    /**
     * Gets distance in meters to the radio source.
     *
     * @return distance in mters to the radio source.
     */
    public double getDistance() {
        return mDistance;
    }

    /**
     * Gets standard deviation of distance, if available.
     *
     * @return standard deviation of distance or null.
     */
    public Double getDistanceStandardDeviation() {
        return mDistanceStandardDeviation;
    }

    /**
     * Gets number of attempted measurements used in the RTT exchange.
     *
     * @return number of attempted measurements used in the RTT exchange.
     */
    public int getNumAttemptedMeasurements() {
        return mNumAttemptedMeasurements;
    }

    /**
     * Gets number of successful measurements used to calculate the distance and
     * standard deviation.
     *
     * @return number of successful measurements used to calculate the distance and
     * standard deviation.
     */
    public int getNumSuccessfulMeasurements() {
        return mNumSuccessfulMeasurements;
    }

    /**
     * Gets received signal strength indicator of (of this 802.11 network for a WiFi access point or
     * of this bluetooth beacon), in dBm.
     *
     * @return received signal strength indicator.
     */
    public double getRssi() {
        return mRssi;
    }

    /**
     * Gets standard deviation of RSSI, if available.
     *
     * @return standard deviation of RSSI, if available.
     */
    public Double getRssiStandardDeviation() {
        return mRssiStandardDeviation;
    }
}
