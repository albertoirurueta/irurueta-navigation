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
 * Contains a ranging reading associated to a given radio source (e.g. WiFi
 * access point or bluetooth beacon), indicating the distance to such source.
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("WeakerAccess")
public class RangingReading<S extends RadioSource> extends Reading<S> {

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
     * Number of attempted measurements used in the RTT exchange.
     */
    private int mNumAttemptedMeasurements = DEFAULT_NUM_MEASUREMENTS;

    /**
     * Number of successful measurements used to calculate the distance and standard
     * deviation.
     */
    private int mNumSuccessfulMeasurements = DEFAULT_NUM_MEASUREMENTS;

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @throws IllegalArgumentException if radio source data is null or distance is negative.
     */
    public RangingReading(S source, double distance) {
        super(source);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        mDistance = distance;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * number of attempted measures is less than 1 or number of successful measures is negative.
     */
    public RangingReading(S source, double distance, int numAttemptedMeasurements,
                          int numSuccessfulMeasurements) {
        this(source, distance);

        if (numAttemptedMeasurements < DEFAULT_NUM_MEASUREMENTS ||
                numSuccessfulMeasurements < 0) {
            throw new IllegalArgumentException();
        }

        mNumAttemptedMeasurements = numAttemptedMeasurements;
        mNumSuccessfulMeasurements = numSuccessfulMeasurements;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative or
     * standard deviation is zero or negative.
     */
    public RangingReading(S source, double distance,
                          Double distanceStandardDeviation) {
        this(source, distance);

        if (distanceStandardDeviation != null && distanceStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }

        mDistanceStandardDeviation = distanceStandardDeviation;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative or
     * standard deviation is zero or negative, number of attempted measures is less
     * than 1 or number of successful measures is negative.
     */
    public RangingReading(S source, double distance,
                          Double distanceStandardDeviation,
                          int numAttemptedMeasurements,
                          int numSuccessfulMeasurements) {
        this(source, distance, distanceStandardDeviation);

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
    protected RangingReading() {
        super();
    }

    /**
     * Contains radio source reading type.
     * @return reading type.
     */
    @Override
    public ReadingType getType() {
        return ReadingType.RANGING_READING;
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
     * Gets number of attempted measurements used in the RTT exchange.
     * @return number of attempted measurements used in the RTT exchange.
     */
    public int getNumAttemptedMeasurements() {
        return mNumAttemptedMeasurements;
    }

    /**
     * Gets number of successful measurements used to calculate the distance and
     * standard deviation.
     * @return number of successful measurements used to calculate the distance and
     * standard deviation.
     */
    public int getNumSuccessfulMeasurements() {
        return mNumSuccessfulMeasurements;
    }
}
