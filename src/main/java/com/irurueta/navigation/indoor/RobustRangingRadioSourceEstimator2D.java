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
import com.irurueta.navigation.NavigationException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 2D position of a radio source (e.g. WiFi
 * access point or bluetooth beacon), by discarding outliers.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustRangingRadioSourceEstimator2D<S extends RadioSource> extends
        RobustRangingRadioSourceEstimator<S, Point2D> {

    /**
     * Radio source estimator used internally.
     */
    protected RangingRadioSourceEstimator2D<S> mInnerEstimator =
            new RangingRadioSourceEstimator2D<>();

    /**
     * Subset of readings used by inner estimator.
     */
    private List<RangingReadingLocated<S, Point2D>> mInnerReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    public RobustRangingRadioSourceEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator2D(
            List<? extends RangingReadingLocated<S, Point2D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator2D(
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator2D(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public RobustRangingRadioSourceEstimator2D(Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator2D(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition) throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator2D(Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator2D(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>();
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>();
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>();
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>();
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>();
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(readings);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            Point2D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>();
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>();
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>();
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.**
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            Point2D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, initialPosition);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores, Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(initialPosition,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 2D position radio source estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 2D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator2D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point2D>> readings,
            Point2D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator2D<>(readings,
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator2D<>(qualityScores,
                        readings, initialPosition, listener);
        }
    }

    /**
     * Gets minimum required number of readings to estimate position of radio source,
     * which is 3 readings.
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 2.
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source.
     * @return estimated located radio source or null.
     */
    @SuppressWarnings("unchecked")
    @Override
    public RadioSourceLocated<Point2D> getEstimatedRadioSource() {
        List<? extends RangingReadingLocated<S, Point2D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        S source = readings.get(0).getSource();

        Point2D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        if (source instanceof WifiAccessPoint) {
            WifiAccessPoint accessPoint = (WifiAccessPoint)source;
            return new WifiAccessPointLocated2D(accessPoint.getBssid(),
                    accessPoint.getFrequency(), accessPoint.getSsid(),
                    estimatedPosition, estimatedPositionCovariance);
        } else if (source instanceof Beacon) {
            Beacon beacon = (Beacon)source;
            return new BeaconLocated2D(beacon.getIdentifiers(),
                    beacon.getTransmittedPower(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(), estimatedPosition,
                    estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Solves preliminar solution for a subset of samples.
     * @param samplesIndices indices of subset samples.
     * @param solutions instance where solution will be stored.
     */
    @Override
    protected void solvePreliminarSolutions(int[] samplesIndices,
            List<Solution<Point2D>> solutions) {
        try {
            int index;

            mInnerReadings.clear();
            for (int samplesIndice : samplesIndices) {
                index = samplesIndice;
                mInnerReadings.add(mReadings.get(index));
            }

            //initial position might or might not be available
            mInnerEstimator.setInitialPosition(mInitialPosition);

            mInnerEstimator.setReadings(mInnerReadings);

            //for preliminar solutions, non linear solver is not needed, and if no
            //initial position is used, we can obtain faster solutions disabling
            //non-linear solver and using a linear one only (because covariance is not
            //required)
            mInnerEstimator.setNonLinearSolverEnabled(mInitialPosition != null);

            mInnerEstimator.estimate();

            Point2D estimatedPosition = mInnerEstimator.getEstimatedPosition();
            solutions.add(new Solution<>(estimatedPosition));
        } catch(NavigationException ignore) {
            //if anything fails, no solution is added
        }
    }

    /**
     * Attempts to refine estimated position and transmitted power contained in
     * provided solution if refinement is requested.
     * This method sets a refined result and transmitted power or provided input
     * result if refinement is not requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined result.
     * solution if not requested or refinement failed.
     * @param result result to be refined.
     */
    protected void attemptRefine(Solution<Point2D> result) {
        Point2D initialPosition = result.getEstimatedPosition();

        if (mRefineResult && mInliersData != null) {
            BitSet inliers = mInliersData.getInliers();
            int nSamples = mReadings.size();

            mInnerReadings.clear();

            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    //sample is inlier
                    mInnerReadings.add(mReadings.get(i));
                }
            }

            try {
                mInnerEstimator.setInitialPosition(initialPosition);
                mInnerEstimator.setReadings(mInnerReadings);

                mInnerEstimator.setNonLinearSolverEnabled(true);
                mInnerEstimator.estimate();

                Matrix cov = mInnerEstimator.getEstimatedCovariance();
                if (mKeepCovariance && cov != null) {
                    //keep covariance
                    mEstimatedPositionCovariance = mCovariance = cov;

                } else {
                    mCovariance = null;
                    mEstimatedPositionCovariance = null;
                }

                mEstimatedPosition = mInnerEstimator.getEstimatedPosition();
            } catch (Exception e) {
                //refinement failed, so we return input value, and covariance
                //becomes unavailable
                mCovariance = null;
                mEstimatedPositionCovariance = null;

                mEstimatedPosition = initialPosition;
            }
        } else {
            mCovariance = null;
            mEstimatedPositionCovariance = null;

            mEstimatedPosition = initialPosition;
        }
    }
}
