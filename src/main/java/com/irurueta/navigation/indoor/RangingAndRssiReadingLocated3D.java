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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;

/**
 * Contains a 3D located reading associated to a given radio source (e.g. WiFi access point
 * or bluetooth beacon) containing signal strength and distance to associated access point.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RangingAndRssiReadingLocated3D<S extends RadioSource> extends
        RangingAndRssiReadingLocated<S, Point3D> {

    /**
     * Constructor.
     *
     * @param source   radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param rssi     received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     *                                  or position is null.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position) {
        super(source, distance, rssi, position);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null, number of attempted measures is less than 1 or number of
     *                                  successful measures is negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final int numAttemptedMeasurements,
            final int numSuccessfulMeasurements) {
        super(source, distance, rssi, position, numAttemptedMeasurements,
                numSuccessfulMeasurements);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null or any of the standard deviations is zero or negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Double distanceStandardDeviation,
            final Double rssiStandardDeviation) {
        super(source, distance, rssi, position, distanceStandardDeviation,
                rssiStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI, if available.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null, any of the standard deviations is zero or negative, number of
     *                                  attempted measures is less than 1 or number of successful measures is negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Double distanceStandardDeviation,
            final Double rssiStandardDeviation, final int numAttemptedMeasurements,
            final int numSuccessfulMeasurements) {
        super(source, distance, rssi, position, distanceStandardDeviation,
                rssiStandardDeviation, numAttemptedMeasurements,
                numSuccessfulMeasurements);
    }

    /**
     * Constructor.
     *
     * @param source             radio source associated to this reading.
     * @param distance           distance in meters to the radio source.
     * @param rssi               received signal strength indicator in dBm.
     * @param position           position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     *                                  or position is null.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Matrix positionCovariance) {
        super(source, distance, rssi, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param positionCovariance        covariance of inhomogeneous coordinates of
     *                                  current position (if available).
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null, position covariance has wrong size, number of attempted
     *                                  measures is less than 1 or number of successful measures is negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Matrix positionCovariance,
            final int numAttemptedMeasurements,
            final int numSuccessfulMeasurements) {
        super(source, distance, rssi, position, positionCovariance,
                numAttemptedMeasurements, numSuccessfulMeasurements);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI value.
     * @param positionCovariance        covariance of inhomogeneous coordinates of
     *                                  current position (if available).
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null or standard deviation is zero or negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Double distanceStandardDeviation,
            final Double rssiStandardDeviation,
            final Matrix positionCovariance) {
        super(source, distance, rssi, position, distanceStandardDeviation,
                rssiStandardDeviation, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param rssi                      received signal strength indicator in dBm.
     * @param position                  position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param rssiStandardDeviation     standard deviation of RSSI value.
     * @param positionCovariance        covariance of inhomogeneous coordinates of
     *                                  current position (if available).
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  position is null, any of the standard deviations is zero or negative,
     *                                  position covariance has wrong size, number of attempted
     *                                  measures is less than 1 or number of successful measures is negative.
     */
    public RangingAndRssiReadingLocated3D(
            final S source, final double distance, final double rssi,
            final Point3D position, final Double distanceStandardDeviation,
            final Double rssiStandardDeviation, final Matrix positionCovariance,
            final int numAttemptedMeasurements, final int numSuccessfulMeasurements) {
        super(source, distance, rssi, position, distanceStandardDeviation,
                rssiStandardDeviation, positionCovariance, numAttemptedMeasurements,
                numSuccessfulMeasurements);
    }

    /**
     * Empty constructor.
     */
    protected RangingAndRssiReadingLocated3D() {
        super();
    }
}

