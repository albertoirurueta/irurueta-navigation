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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Base class for 2D robust position estimators using located radio sources and their
 * readings at unknown locations.
 * These kind of estimators can be used to robustly determine the 2D position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * 2D locations are known.
 * Implementations of this class should be able to detect and discard outliers in order
 * to find the best solution.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustPositionEstimator2D extends RobustPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public RobustPositionEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public RobustPositionEstimator2D(RobustPositionEstimatorListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     * @return minimum required number of located radio sources to perform trilateration.
     */
    public int getMinRequiredSources() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Creates a robust 2D position estimator.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D();
            case LMedS:
                return new LMedSRobustPositionEstimator2D();
            case MSAC:
                return new MSACRobustPositionEstimator2D();
            case PROSAC:
                return new PROSACRobustPositionEstimator2D();
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D();
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param sources located radio sources used for trilateration.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number
     * of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(sources);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(sources);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(fingerprint);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(fingerprint);
            case MSAC:
                return new MSACRobustPositionEstimator2D(fingerprint);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(fingerprint);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources, fingerprint);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources, fingerprint);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources, fingerprint);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(sources, fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(sources, fingerprint);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources, listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources, listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources, listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(sources, listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(sources, listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided location radio sources.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(fingerprint, listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(fingerprint, listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(fingerprint, listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(fingerprint, listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param source located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint
     * is null or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> source,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(source, fingerprint,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D();
            case LMedS:
                return new LMedSRobustPositionEstimator2D();
            case MSAC:
                return new MSACRobustPositionEstimator2D();
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number
     * of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        sources);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        sources);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(fingerprint);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(fingerprint);
            case MSAC:
                return new MSACRobustPositionEstimator2D(fingerprint);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        fingerprint);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources, fingerprint);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources, fingerprint);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources, fingerprint);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        sources, fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        sources, fingerprint);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(sources, listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(sources, listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(sources, listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        sources, listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        sources, listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided location radio sources.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(fingerprint, listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(fingerprint, listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(fingerprint, listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        fingerprint, listener);
        }
    }

    /**
     * Creates a robust 2D position estimator.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param source located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint
     * is null or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> source,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener,
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case LMedS:
                return new LMedSRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case MSAC:
                return new MSACRobustPositionEstimator2D(source, fingerprint,
                        listener);
            case PROSAC:
                return new PROSACRobustPositionEstimator2D(qualityScores,
                        source, fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustPositionEstimator2D(qualityScores,
                        source, fingerprint, listener);
        }
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param sources located radio sources used for trilateration.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number
     * of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources) {
        return create(sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        return create(fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        return create(sources, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided location radio sources.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param source located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint
     * is null or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(
            List<? extends RadioSourceLocated<Point2D>> source,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(source, fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number
     * of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources) {
        return create(qualityScores, sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        return create(qualityScores, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     * or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        return create(qualityScores, sources, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     * provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> sources,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(qualityScores, sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided location radio sources.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(qualityScores, fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator using default method.
     * @param qualityScores quality scores corresponding to each provided
     *                     located radio source. The larger the score
     *                     value the better the quality of the radio
     *                     source.
     * @param source located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint
     * is null or the number of provided sources is less than the required minimum.
     */
    public static RobustPositionEstimator2D create(double[] qualityScores,
            List<? extends RadioSourceLocated<Point2D>> source,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            RobustPositionEstimatorListener<Point2D> listener) {
        return create(qualityScores, source, fingerprint, listener,
                DEFAULT_ROBUST_METHOD);
    }


    /**
     * Sets positions, distances and standard deviations of distances on internal trilateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     * @param distanceQualityScores distance quality scores or null if not required.
     */
    @Override
    protected void setPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            List<Point2D> positions, List<Double> distances,
            List<Double> distanceStandardDeviations,
            List<Double> distanceQualityScores) {

        int size = positions.size();
        Point2D[] positionsArray = new InhomogeneousPoint2D[size];
        positionsArray = positions.toArray(positionsArray);

        double[] distancesArray = new double[size];
        double[] distanceStandardDeviationsArray = new double[size];

        double[] qualityScoresArray = null;
        if (distanceQualityScores != null) {
            qualityScoresArray = new double[size];
        }

        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);

            if (qualityScoresArray != null) {
                qualityScoresArray[i] = distanceQualityScores.get(i);
            }
        }

        try {
            mTrilaterationSolver.setPositionsDistancesAndStandardDeviations(
                    positionsArray, distancesArray, distanceStandardDeviationsArray);

            if (qualityScoresArray != null) {
                mTrilaterationSolver.setQualityScores(qualityScoresArray);
            }
        } catch (LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }
}
