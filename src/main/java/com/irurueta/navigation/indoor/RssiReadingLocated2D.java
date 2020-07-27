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
import com.irurueta.geometry.Point2D;

/**
 * Contains a 2D located signal strength reading associated to a given radio source
 * (e.g. WiFi access point or bluetooth beacon).
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RssiReadingLocated2D<S extends RadioSource> extends
        RssiReadingLocated<S, Point2D> {

    /**
     * Constructor.
     *
     * @param source   radio source associated to this reading.
     * @param rssi     received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if either radio source data or position
     *                                  are null.
     */
    public RssiReadingLocated2D(
            final S source, final double rssi, final Point2D position) {
        super(source, rssi, position);
    }

    /**
     * Constructor.
     *
     * @param source                radio source associated to this reading.
     * @param rssi                  received signal strength indicator in dBm.
     * @param position              position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if either radio source data or position
     *                                  are null.
     */
    public RssiReadingLocated2D(
            final S source, final double rssi, final Point2D position,
            final Double rssiStandardDeviation) {
        super(source, rssi, position, rssiStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param source             radio source associated to this reading.
     * @param rssi               received signal strength indicator in dBm.
     * @param position           position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either radio source or position are
     *                                  null, or covariance has invalid size.
     */
    public RssiReadingLocated2D(
            final S source, final double rssi, final Point2D position,
            final Matrix positionCovariance) {
        super(source, rssi, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param source                radio source associated to this reading.
     * @param rssi                  received signal strength indicator in dBm.
     * @param position              position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @param positionCovariance    covariance of inhomogeneous coordinates of
     *                              current position (if available).
     * @throws IllegalArgumentException if either radio source or position are
     *                                  null, or covariance has invalid size.
     */
    public RssiReadingLocated2D(
            final S source, final double rssi, final Point2D position,
            final Double rssiStandardDeviation,
            final Matrix positionCovariance) {
        super(source, rssi, position, rssiStandardDeviation,
                positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected RssiReadingLocated2D() {
        super();
    }
}
