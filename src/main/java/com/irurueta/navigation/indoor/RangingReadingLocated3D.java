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
 * Contains a 3D located ranging reading associated to a given radio source
 * (e.g. WiFi access point or bluetooth beacon), indicating the distance
 * to such radio source.
 * @param <S> a {@link RadioSource} type.
 */
public class RangingReadingLocated3D<S extends RadioSource> extends RangingReadingLocated<S, Point3D> {

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     * or position is null.
     */
    public RangingReadingLocated3D(S source, double distance,
            Point3D position) throws IllegalArgumentException {
        super(source, distance, position);
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null or standard deviation is zero or negative.
     */
    public RangingReadingLocated3D(S source, double distance,
            Point3D position, Double distanceStandardDeviation)
            throws IllegalArgumentException {
        super(source, distance, position, distanceStandardDeviation);
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     * or position is null.
     */
    public RangingReadingLocated3D(S source, double distance,
            Point3D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(source, distance, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null or standard deviation is zero or negative.
     */
    public RangingReadingLocated3D(S source, double distance,
            Point3D position, Double distanceStandardDeviation,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(source, distance, position, distanceStandardDeviation,
                positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected RangingReadingLocated3D() {
        super();
    }
}
