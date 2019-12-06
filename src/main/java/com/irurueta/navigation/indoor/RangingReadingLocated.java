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
 * Contains a located ranging reading associated to a given radio source (e.g. WiFi access point or
 * bluetooth beacon), indicating the distance to such access point.
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
public class RangingReadingLocated<S extends RadioSource, P extends Point<?>> extends RangingReading<S>
        implements ReadingLocated<P> {

    /**
     * Position where WiFi reading was made.
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
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @throws IllegalArgumentException if radio source data is null, distance is negative
     * or position is null.
     */
    public RangingReadingLocated(S source, double distance,
            P position) {
        super(source, distance);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null, number of attempted measures is less than 1 or number of successful
     * measures is negative.
     */
    public RangingReadingLocated(S source, double distance,
            P position, int numAttemptedMeasurements, int numSuccessfulMeasurements) {
        super(source, distance, numAttemptedMeasurements, numSuccessfulMeasurements);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
    public RangingReadingLocated(S source, double distance,
            P position, Double distanceStandardDeviation) {
        super(source, distance, distanceStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null or standard deviation is zero or negative, number of attempted
     * measures is less than 1 or number of successful measures is negative.
     */
    public RangingReadingLocated(S source, double distance,
            P position, Double distanceStandardDeviation,
            int numAttemptedMeasurements, int numSuccessfulMeasurements) {
        super(source, distance, distanceStandardDeviation,
                numAttemptedMeasurements, numSuccessfulMeasurements);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null or position covariance has wrong size.
     */
    public RangingReadingLocated(S source, double distance,
            P position, Matrix positionCovariance) {
        this(source, distance, position);

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
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null, position covariance has wrong size, number of attempted
     * measures is less than 1 or number of successful measures is negative.
     */
    public RangingReadingLocated(S source, double distance, P position,
            Matrix positionCovariance, int numAttemptedMeasurements,
            int numSuccessfulMeasurements) {
        super(source, distance, numAttemptedMeasurements, numSuccessfulMeasurements);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }

        mPosition = position;
        mPositionCovariance = positionCovariance;
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
    public RangingReadingLocated(S source, double distance,
            P position, Double distanceStandardDeviation,
            Matrix positionCovariance) {
        this(source, distance, position, distanceStandardDeviation);

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
     * @param distance distance in meters to the radio source.
     * @param position position where reading was made.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param positionCovariance covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @param numAttemptedMeasurements number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     * position is null, distance standard deviation is zero or negative, position
     * covariance has wrong size, number of attempted measures is less than 1 or
     * number of successful measures is negative.
     */
    public RangingReadingLocated(S source, double distance, P position,
            Double distanceStandardDeviation, Matrix positionCovariance,
            int numAttemptedMeasurements, int numSuccessfulMeasurements) {
        super(source, distance, distanceStandardDeviation,
                numAttemptedMeasurements, numSuccessfulMeasurements);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }

        mPosition = position;
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected RangingReadingLocated() {
        super();
    }

    /**
     * Gets position where reading was made.
     * @return position where reading was made.
     */
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     * @return covariance of position or null.
     */
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
