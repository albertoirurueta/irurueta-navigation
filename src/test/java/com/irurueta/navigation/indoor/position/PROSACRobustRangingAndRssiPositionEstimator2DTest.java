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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;
import com.irurueta.navigation.lateration.PROSACRobustLateration2DSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.junit.Test;

public class PROSACRobustRangingAndRssiPositionEstimator2DTest implements
        RobustRangingAndRssiPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            PROSACRobustRangingAndRssiPositionEstimator2DTest.class.getName());

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
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // empty constructor
        PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources
        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with fingerprints
        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(fingerprint);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with sources and fingerprint
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources,
                fingerprint);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources,
                this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with fingerprint and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(fingerprint,
                this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources,
                fingerprint, this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores
        final double[] sourceQualityScores = new double[3];
        final double[] fingerprintReadingsQualityScores = new double[3];
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1],
                    fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores and sources
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores,
                fingerprintReadingsQualityScores, sources);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null,
                    fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (List<WifiAccessPointLocated2D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores and fingerprints
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, fingerprint);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores, sources and fingerprint
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources,
                fingerprint);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, sources,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores, sources and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources,
                this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1],
                    fingerprintReadingsQualityScores, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    new double[1], sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (List<WifiAccessPointLocated2D>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with quality scores, fingerprint and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores,
                fingerprintReadingsQualityScores, fingerprint, this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources,
                fingerprint, this);

        // check default values
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertEquals(estimator.getPreliminarySubsetSize(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores,
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null, sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], sources, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getThreshold(),
                PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, 0.0);

        // set new value
        estimator.setThreshold(1.0);

        // check
        assertEquals(estimator.getThreshold(), 1.0, 0.0);

        // force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResiduals() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertFalse(estimator.isComputeAndKeepResiduals());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetSources() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(estimator.getSources(), sources);

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(estimator.getFingerprint(), fingerprint);

        // force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D solver =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        final Point2D p = Point2D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(solver.getInitialPosition(), p);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);

        // set new value
        estimator.setFallbackDistanceStandardDeviation(1.0);

        // check
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                1.0, 0.0);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

        // force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(100);

        // check
        assertEquals(estimator.getMaxIterations(), 100);

        // force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isLinearSolverUsed());

        // set new value
        estimator.setLinearSolverUsed(false);

        // check
        assertFalse(estimator.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isPreliminarySolutionRefined());

        // set new value
        estimator.setPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetSourceQualityScores() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        final double[] qualityScores = new double[3];
        estimator.setSourceQualityScores(qualityScores);

        // check
        assertSame(estimator.getSourceQualityScores(), qualityScores);

        // force IllegalArgumentException
        try {
            estimator.setSourceQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setSourceQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        final double[] qualityScores = new double[3];
        estimator.setFingerprintReadingsQualityScores(qualityScores);

        // check
        assertSame(estimator.getFingerprintReadingsQualityScores(), qualityScores);

        // force IllegalArgumentException
        try {
            estimator.setFingerprintReadingsQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setFingerprintReadingsQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetEvenlyDistributeReadings() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.getEvenlyDistributeReadings());

        // set new value
        estimator.setEvenlyDistributeReadings(false);

        // check
        assertFalse(estimator.getEvenlyDistributeReadings());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                spy(new PROSACRobustRangingAndRssiPositionEstimator2D());

        // check default value
        assertEquals(estimator.getPreliminarySubsetSize(), 3);

        // set new value
        estimator.setPreliminarySubsetSize(4);

        // check
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        verify(estimator, times(1))
                .buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();

        // force IllegalArgumentException
        try {
            estimator.setPreliminarySubsetSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testEstimate() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMultipleReadingsPerSource() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierError() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMultipleReadingsPerSourceWithInlierError()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setHomogeneousLinearSolverUsed(true);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setHomogeneousLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
    public void testEstimatePreliminaryNotRefined()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
    public void testEstimateLinearDisabled()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(
                            sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);
            estimator.setInitialPosition(position);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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
    public void testEstimateLargerPreliminarySubsetSize() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                    new PROSACRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);
            estimator.setPreliminarySubsetSize(4);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            final Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            numValidPosition++;
            break;
        }

        assertTrue(numValidPosition > 0);

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final PROSACRobustRangingAndRssiPositionEstimator2D estimator =
                new PROSACRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(
            final RobustRangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((PROSACRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(
            final RobustRangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((PROSACRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingAndRssiPositionEstimator<Point2D> estimator,
            final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingAndRssiPositionEstimator<Point2D> estimator,
            final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private double receivedPower(final double equivalentTransmittedPower,
                                 final double distance, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI *
                        PROSACRobustRangingAndRssiPositionEstimator2DTest.FREQUENCY),
                pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final PROSACRobustRangingAndRssiPositionEstimator2D estimator) {
        try {
            estimator.setPreliminarySubsetSize(3);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEvenlyDistributeReadings(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
