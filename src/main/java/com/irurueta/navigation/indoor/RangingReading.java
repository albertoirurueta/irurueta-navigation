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
public class RangingReading<S extends RadioSource> extends Reading<S> {

    /**
     * Distance in meters to the radio source.
     */
    private double mDistance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double mDistanceStandardDeviation;

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
}
