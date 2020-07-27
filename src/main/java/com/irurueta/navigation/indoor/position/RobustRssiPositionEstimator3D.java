/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Base class for robust 3D RSSI position estimators using located radio sources and their
 * RSSI readings at unknown locations.
 * These kind of estimators can be used to robustly determine the 3D position of a given
 * device by getting RSSI readings at an unknown location of different radio sources
 * whose locations are known.
 * Implementations of this class should be able to detect and discard outliers in order
 * to find the best solution.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class RobustRssiPositionEstimator3D extends
        RobustRssiPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public RobustRssiPositionEstimator3D() {
        super();
        mPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustRssiPositionEstimator3D(
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
        mPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform lateration.
     */
    @Override
    public int getMinRequiredSources() {
        return Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH;
    }


    /**
     * Creates a robust 3D position estimator.
     *
     * @param method robust estimator method.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D();
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D();
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D();
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D();
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D();
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources located radio sources used for lateration.
     * @param method  robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(sources);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(sources);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param method      robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(fingerprint);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(fingerprint);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(fingerprint);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(fingerprint);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param method      robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources, fingerprint);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources, fingerprint);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources, fingerprint);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(sources, fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(sources, fingerprint);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param listener listener in charge of handling events.
     * @param method   robust estimator method.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @param method   robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources,
                        listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources,
                        listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources,
                        listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(sources,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(sources,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @param method      robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(fingerprint,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @param method      robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D();
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D();
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D();
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(fingerprint);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(fingerprint);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(fingerprint);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        fingerprint);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources, fingerprint);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources, fingerprint);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources, fingerprint);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, fingerprint);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, fingerprint);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources,
                        listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources,
                        listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources,
                        listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(fingerprint,
                        listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        fingerprint, listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing ranging+RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case LMedS:
                return new LMedSRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case MSAC:
                return new MSACRobustRssiPositionEstimator3D(sources,
                        fingerprint, listener);
            case PROSAC:
                return new PROSACRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, fingerprint, listener);
            case PROMedS:
            default:
                return new PROMedSRobustRssiPositionEstimator3D(
                        sourceQualityScores, fingerprintReadingQualityScores,
                        sources, fingerprint, listener);
        }
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources located radio sources used for lateration.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources) {
        return create(sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        return create(fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        return create(sources, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param listener listener in charge of handling events.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sources, fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param listener                        listener in charge of handling events.
     * @return a robust 3D position estimator.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param listener                        listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @return a robust 3D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRssiPositionEstimator3D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint,
            final RobustRssiPositionEstimatorListener<Point3D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
     * @param positions                  positions to be set.
     * @param distances                  distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     * @param distanceQualityScores      distance quality scores or null if not
     *                                   required.
     */
    @Override
    protected void setPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            final List<Point3D> positions, List<Double> distances,
            final List<Double> distanceStandardDeviations,
            final List<Double> distanceQualityScores) {
        final int size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        final double[] distancesArray = new double[size];
        final double[] distanceStandardDeviationsArray = new double[size];

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
            mLaterationSolver.setPositionsDistancesAndStandardDeviations(
                    positionsArray, distancesArray, distanceStandardDeviationsArray);

            if (qualityScoresArray != null) {
                mLaterationSolver.setQualityScores(qualityScoresArray);
            }
        } catch (final LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }
}
