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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 3D position of a radio source (e.g. WiFi
 * access point or bluetooth beacon), by discarding outliers.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustRangingRadioSourceEstimator3D<S extends RadioSource> extends
        RobustRangingRadioSourceEstimator<S, Point3D> {

    /**
     * Radio source estimator used internally.
     */
    protected RangingRadioSourceEstimator3D<S> mInnerEstimator =
            new RangingRadioSourceEstimator3D<>();

    /**
     * Subset of readings used by inner estimator.
     */
    private List<RangingReadingLocated<S, Point3D>> mInnerReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    public RobustRangingRadioSourceEstimator3D() {
        super();
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings) {
        super(readings);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator3D(
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public RobustRangingRadioSourceEstimator3D(Point3D initialPosition) {
        super(initialPosition);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings          radio signal readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation of radio
     *                          source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition) {
        super(readings, initialPosition);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition   initial position to start the estimation of radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator3D(Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
        mPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings          radio signal ranging readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation of radio source
     *                          position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
        mPreliminarySubsetSize = getMinReadings();
    }


    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param method robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>();
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>();
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>();
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>();
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>();
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings  radio signal ranging readings belonging to the same radio source.
     * @param method    robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(readings);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param listener  listener in charge of attending events raised by this instance.
     * @param method    robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings  radio signal ranging readings belonging to the same radio source.
     * @param listener  listener in charge of attending events raised by this instance.
     * @param method    robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            Point3D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings          radio signal ranging readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings          radio signal ranging readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>();
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>();
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>();
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param readings      radio signal ranging readings belonging to the same radio source.
     * @param method        robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.**
     * @param readings      radio signal ranging readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sample. The larger the score value the better
     *                          the quality of the sample.*
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            Point3D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        initialPosition);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sample. The larger the score value the better
     *                          the quality of the sample.*
     * @param readings          radio signal ranging readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition, RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, initialPosition);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, initialPosition);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sample. The larger the score value the better
     *                          the quality of the sample.*
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores, Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition,
                        listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        initialPosition, listener);
        }
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sample. The larger the score value the better
     *                          the quality of the sample.*
     * @param readings          radio signal ranging readings belonging to the same radio source.
     * @param initialPosition   initial position to start the estimation or radio
     *                          source position.
     * @param listener          listener in charge of attending events raised by this instance.
     * @param method            robust estimator method.
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            double[] qualityScores,
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case LMedS:
                return new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case MSAC:
                return new MSACRobustRangingRadioSourceEstimator3D<>(readings,
                        initialPosition, listener);
            case PROSAC:
                return new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, initialPosition, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores,
                        readings, initialPosition, listener);
        }
    }

    /**
     * Gets minimum required number of readings to estimate position of radio source,
     * which is 4 readings.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 3.
     *
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source.
     *
     * @return estimated located radio source or null.
     */
    @SuppressWarnings("unchecked")
    @Override
    public RadioSourceLocated<Point3D> getEstimatedRadioSource() {
        List<? extends RangingReadingLocated<S, Point3D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        S source = readings.get(0).getSource();

        Point3D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        if (source instanceof WifiAccessPoint) {
            WifiAccessPoint accessPoint = (WifiAccessPoint)source;
            return new WifiAccessPointLocated3D(accessPoint.getBssid(),
                    accessPoint.getFrequency(), accessPoint.getSsid(),
                    estimatedPosition, estimatedPositionCovariance);
        } else if (source instanceof Beacon) {
            Beacon beacon = (Beacon)source;
            return new BeaconLocated3D(beacon.getIdentifiers(),
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
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    @Override
    public boolean isHomogeneousLinearSolverUsed() {
        return mInnerEstimator.isHomogeneousLinearSolverUsed();
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    @Override
    public void setHomogeneousLinearSolverUsed(
            boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInnerEstimator.setHomogeneousLinearSolverUsed(useHomogeneousLinearSolver);
    }

    /**
     * Solves preliminar solution for a subset of samples.
     *
     * @param samplesIndices    indices of subset samples.
     * @param solutions         instance where solution will be stored.
     */
    @Override
    protected void solvePreliminarSolutions(int[] samplesIndices,
            List<Solution<Point3D>> solutions) {
        try {
            int index;

            mInnerReadings.clear();
            for (int samplesIndice : samplesIndices) {
                index = samplesIndice;
                mInnerReadings.add(mReadings.get(index));
            }

            // initial position might or might not be available
            mInnerEstimator.setInitialPosition(mInitialPosition);

            mInnerEstimator.setReadings(mInnerReadings);

            // for preliminar solutions, non linear solver is not needed, and if no
            // initial position is used, we can obtain faster solutions disabling
            // non-linear solver and using a linear one only (because covariance is not
            // required)
            mInnerEstimator.setNonLinearSolverEnabled(mInitialPosition != null);

            // indicastes whether readings position covariances must be taken into account
            mInnerEstimator.setUseReadingPositionCovariances(mUseReadingPositionCovariances);

            mInnerEstimator.estimate();

            Point3D estimatedPosition = mInnerEstimator.getEstimatedPosition();
            solutions.add(new Solution<>(estimatedPosition));
        } catch (NavigationException ignore) {
            // if anything fails, no solution is added
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
     *
     * @param result result to be refined.
     */
    protected void attemptRefine(Solution<Point3D> result) {
        Point3D initialPosition = result.getEstimatedPosition();

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
                mInnerEstimator.setUseReadingPositionCovariances(
                        mUseReadingPositionCovariances);
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

