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

import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class SequentialRobustRangingAndRssiPositionEstimator3DTest implements
        SequentialRobustRangingAndRssiPositionEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustRangingAndRssiPositionEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 100;
    private static final int MAX_SOURCES = 500;

    private static final int NUM_READINGS = 5;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 0.5;

    private static final int TIMES = 400;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private static final double INLIER_ERROR_STD = 0.1;

    private static final double RANGING_STD = 1.0;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATHLOSS_EXPONENT_VARIANCE = 0.001;

    private int estimateStart;
    private int estimateEnd;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // empty constructor
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());


        // test constructor with sources
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(sources);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    (List<? extends RadioSourceLocated<Point3D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with fingerprint
        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                fingerprint);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with sources and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sources, fingerprint);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(sources,
                    (RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());


        // test constructor with sources and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(sources,
                this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    (List<? extends RadioSourceLocated<Point3D>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                fingerprint, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with sources, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sources, fingerprint, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(null,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores
        double[] sourceQualityScores = new double[4];
        double[] fingerprintReadingsQualityScores = new double[4];
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores and sources
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                sources);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (List<? extends RadioSourceLocated<Point3D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores, sources and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, sources,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    sources,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores and sources and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                sources, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, sources,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (List<? extends RadioSourceLocated<Point3D>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // test constructor with quality scores, sources, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint, this);

        // check default values
        assertEquals(estimator.getNumberOfDimensions(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(estimator.getMinRequiredSources(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RSSI_ROBUST_METHOD);
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, null,
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores, sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetRangingRobustMethod() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(estimator.getRangingRobustMethod(),
                RobustEstimatorMethod.PROSAC);
    }

    @Test
    public void testGetSetRssiRobustMethod() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_RANGING_ROBUST_METHOD);

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(estimator.getRssiRobustMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testIsSetRangingRadioSourcePositionCovarianceUsed()
            throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRangingRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE);
        assertTrue(estimator.isRangingRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRangingRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRangingRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testIsSetRssiRadioSourcePositionCovarianceUsed()
            throws LockedException {

        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRssiRadioSourcePositionCovarianceUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE);
        assertTrue(estimator.isRssiRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRssiRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRssiRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testIsSetRangingReadingEvenlyDistributed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRangingReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS);
        assertTrue(estimator.isRangingReadingsEvenlyDistributed());

        // set new value
        estimator.setRangingReadingsEvenlyDistributed(false);

        // check
        assertFalse(estimator.isRangingReadingsEvenlyDistributed());
    }

    @Test
    public void testGetSetRssiFallbackDistanceStandardDeviation()
            throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);

        // set new value
        double value = new Random().nextDouble();
        estimator.setRssiFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(),
                value, 0.0);
    }

    @Test
    public void testGetSetRangingFallbackDistanceStandardDeviation()
            throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);

        // set new value
        double value = new Random().nextDouble();
        estimator.setRangingFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(),
                value, 0.0);
    }

    @Test
    public void testIsSetRssiReadingsEvenlyDistributed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRssiReadingsEvenlyDistributed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS);
        assertTrue(estimator.isRssiReadingsEvenlyDistributed());

        // set new value
        estimator.setRssiReadingsEvenlyDistributed(false);

        // check
        assertFalse(estimator.isRssiReadingsEvenlyDistributed());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
    }

    @Test
    public void testGetSetRangingConfidence() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);

        // set new value
        estimator.setRangingConfidence(0.7);

        // check
        assertEquals(estimator.getRangingConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetRssiConfidence() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                0.0);

        // set new value
        estimator.setRssiConfidence(0.8);

        // check
        assertEquals(estimator.getRssiConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetRangingMaxIterations() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setRangingMaxIterations(100);

        // check
        assertEquals(estimator.getRangingMaxIterations(), 100);
    }

    @Test
    public void testGetSetRssiMaxIterations() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setRssiMaxIterations(200);

        // check
        assertEquals(estimator.getRssiMaxIterations(), 200);
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT);
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_KEEP_COVARIANCE);
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetRangingLinearSolverUsed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRangingLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_LINEAR_SOLVER);
        assertTrue(estimator.isRangingLinearSolverUsed());

        // set new value
        estimator.setRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isRangingLinearSolverUsed());
    }

    @Test
    public void testIsSetRssiLinearSolverUsed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRssiLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_LINEAR_SOLVER);
        assertTrue(estimator.isRssiLinearSolverUsed());

        // set new value
        estimator.setRssiLinearSolverUsed(false);

        // check
        assertFalse(estimator.isRssiLinearSolverUsed());
    }

    @Test
    public void testIsSetRangingHomogeneousLinearSolverUsed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRangingHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER);
        assertFalse(estimator.isRangingHomogeneousLinearSolverUsed());

        // set new value
        estimator.setRangingHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isRangingHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetRssiHomogeneousLinearSolverUsed() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRssiHomogeneousLinearSolverUsed(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER);
        assertFalse(estimator.isRssiHomogeneousLinearSolverUsed());

        // set new value
        estimator.setRssiHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isRssiHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetRangingPreliminarySolutionRefined() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRangingPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS);
        assertTrue(estimator.isRangingPreliminarySolutionRefined());

        // set new value
        estimator.setRangingPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isRangingPreliminarySolutionRefined());
    }

    @Test
    public void testIsSetRssiPreliminarySolutionRefined() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(estimator.isRssiPreliminarySolutionRefined(),
                SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS);
        assertTrue(estimator.isRssiPreliminarySolutionRefined());

        // set new value
        estimator.setRssiPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isRssiPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetRangingPreliminarySubsetSize() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        assertEquals(estimator.getRangingPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);

        // set new value
        estimator.setRangingPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setRangingPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        assertEquals(estimator.getRssiPreliminarySubsetSize(),
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1);

        // set new value
        estimator.setRssiPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setRssiPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(estimator.getRangingThreshold(), value, 0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(estimator.getRssiThreshold(), value, 0.0);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator.setSources(sources);

        // check
        assertSame(estimator.getSources(), sources);

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(estimator.getFingerprint(), fingerprint);
    }

    @Test
    public void testGetSetSourceQualityScores() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        double[] value = new double[4];
        estimator.setSourceQualityScores(value);

        // check
        assertSame(estimator.getSourceQualityScores(), value);

        // force IllegalArgumentException
        try {
            estimator.setSourceQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprintReadingsQualityScores()
            throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        double[] value = new double[4];
        estimator.setFingerprintReadingsQualityScores(value);

        // check
        assertSame(estimator.getFingerprintReadingsQualityScores(), value);

        // force IllegalArgumentException
        try {
            estimator.setFingerprintReadingsQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        Point3D p = Point3D.create();
        estimator.setInitialPosition(p);

        // check
        assertSame(estimator.getInitialPosition(), p);
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());


            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateMultipleReadingsPerSource() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);
                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2),
                            rssi + errorRssi1 + errorRssi2,
                            RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierError() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError),
                        rssi + errorRssi1 + errorRssi2 + inlierError,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateMultipleReadingsPerSourceWithInlierError() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);
                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError),
                            rssi + errorRssi1 + errorRssi2 + inlierError,
                            RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined()
            throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiHomogeneousLinearSolverUsed(true);
            estimator.setRangingHomogeneousLinearSolverUsed(true);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateLinearSolverUsedInhomogeneousAndPreliminaryRefined()
            throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiHomogeneousLinearSolverUsed(false);
            estimator.setRangingHomogeneousLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimatePreliminaryNotRefined() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiPreliminarySolutionRefined(false);
            estimator.setRangingPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateLinearDisabled() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateLinearDisabledAndNotPreliminaryRefined()
            throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(false);
            estimator.setRangingPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateLinearDisabledWithInitialPosition()
            throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(
                            sourceQualityScores, fingerprintReadingsQualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);
            estimator.setInitialPosition(position);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateLargerPrelminiarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 /
                        (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        rssi + errorRssi1 + errorRssi2,
                        RANGING_STD,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRangingPreliminarySubsetSize(5);
            estimator.setRssiPreliminarySubsetSize(5);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());


            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        SequentialRobustRangingAndRssiPositionEstimator3D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(
            SequentialRobustRangingAndRssiPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator3D)estimator);
    }

    @Override
    public void onEstimateEnd(
            SequentialRobustRangingAndRssiPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator3D)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            SequentialRobustRangingAndRssiPositionEstimator<Point3D> estimator,
            float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator3D)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private double receivedPower(double equivalentTransmittedPower,
                                 double distance, double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY),
                pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(SequentialRobustRangingAndRssiPositionEstimator3D estimator) {
        try {
            estimator.setRangingRobustMethod(RobustEstimatorMethod.PROMedS);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiRobustMethod(RobustEstimatorMethod.PROMedS);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingReadingsEvenlyDistributed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiReadingsEvenlyDistributed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingConfidence(0.9);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiConfidence(0.9);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingPreliminarySubsetSize(4);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiPreliminarySubsetSize(4);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSourceQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprintReadingsQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
    }
}
