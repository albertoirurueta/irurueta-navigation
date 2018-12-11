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
import com.irurueta.geometry.Point;

/**
 * Contains a located signal strength reading associated to a given radio source
 * (e.g. WiFi access point or bluetooth beacon).
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
public class RssiReadingLocated<S extends RadioSource, P extends Point> extends RssiReading<S>
        implements ReadingLocated<P>{

    /**
     * Position where radio source reading was made.
     */
    private P mPosition;

    /**
     * Covariance of inhomogeneous coordinates of current position
     * (if available).
     */
    private Matrix mPositionCovariance;

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if either radio source data or position
     * are null.
     */
    public RssiReadingLocated(S source, double rssi, P position) {
        super(source, rssi);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if either radio source data or position
     * are null.
     */
    public RssiReadingLocated(S source, double rssi,
            P position, Double rssiStandardDeviation) {
        super(source, rssi, rssiStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either radio source or position are
     * null, or covariance has invalid size.
     */
    public RssiReadingLocated(S source, double rssi,
            P position, Matrix positionCovariance) {
        this(source, rssi, position);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param rssi received signal strength indicator in dBm.
     * @param position position where reading was made.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either radio source or position are
     * null, or covariance has invalid size.
     */
    public RssiReadingLocated(S source, double rssi,
            P position, Double rssiStandardDeviation, Matrix positionCovariance) {
        this(source, rssi, position, rssiStandardDeviation);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected RssiReadingLocated() {
        super();
    }

    /**
     * Gets position where reading was made.
     * @return position where reading was made.
     */
    @Override
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     * @return covariance of position or null.
     */
    @Override
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
