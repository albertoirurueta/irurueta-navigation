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
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.navigation.lateration.PROMedSRobustLateration3DSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
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


public class PROMedSRobustRangingPositionEstimator3DTest implements
        RobustRangingPositionEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            PROMedSRobustRangingPositionEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 100;
    private static final int MAX_SOURCES = 500;

    private static final int NUM_READINGS = 5;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 0.5;

    private static final int TIMES = 400;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private static final double INLIER_ERROR_STD = 0.1;

    private static final double RANGING_STD = 1.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // empty constructor
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator = new PROMedSRobustRangingPositionEstimator3D(sources);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprints
        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        estimator = new PROMedSRobustRangingPositionEstimator3D(fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources and fingerprint
        estimator = new PROMedSRobustRangingPositionEstimator3D(sources, fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(sources,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sources, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(fingerprint,
                this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sources, fingerprint,
                this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(null,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores
        double[] sourceQualityScores = new double[4];
        double[] fingerprintReadingsQualityScores = new double[4];
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores and sources
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (List<WifiAccessPointLocated3D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores and fingerprints
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    new double[1], fingerprintReadingsQualityScores,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores, sources and fingerprint
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores, sources,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores, sources and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null, sources,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (List<WifiAccessPointLocated3D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with quality scores, fingerprint and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, fingerprint, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, fingerprint,
                this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
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
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    null, fingerprintReadingsQualityScores, sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, null, sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(new double[1],
                    fingerprintReadingsQualityScores, sources, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, new double[1], sources, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustRangingPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        estimator.setStopThreshold(1.0);

        // check
        assertEquals(estimator.getStopThreshold(), 1.0, 0.0);

        // force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetSources() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(estimator.getFingerprint(), fingerprint);

        // force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D solver =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        Point3D p = Point3D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(solver.getInitialPosition(), p);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

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
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertEquals(estimator.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

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
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.isLinearSolverUsed());

        // set new value
        estimator.setLinearSolverUsed(false);

        // check
        assertFalse(estimator.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.isPreliminarySolutionRefined());

        // set new value
        estimator.setPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetSourceQualityScores() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        double[] qualityScores = new double[4];
        estimator.setSourceQualityScores(qualityScores);

        // check
        assertSame(estimator.getSourceQualityScores(), qualityScores);

        // force IllegalArgumentException
        try {
            estimator.setSourceQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setSourceQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    public void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        double[] qualityScores = new double[3];
        estimator.setFingerprintReadingsQualityScores(qualityScores);

        // check
        assertSame(estimator.getFingerprintReadingsQualityScores(), qualityScores);

        // force IllegalArgumentException
        try {
            estimator.setFingerprintReadingsQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setFingerprintReadingsQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetEvenlyDistributeReadings() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertTrue(estimator.getEvenlyDistributeReadings());

        // set new value
        estimator.setEvenlyDistributeReadings(false);

        // check
        assertFalse(estimator.getEvenlyDistributeReadings());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();

        // check default value
        assertEquals(estimator.getPreliminarySubsetSize(), 4);

        // set new value
        estimator.setPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testEstimate() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateMultipleReadingsPerSource() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                } else {
                    error1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                for (int j = 0; j < NUM_READINGS; j++) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        error2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        error2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(error2));

                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + error1 + error2),
                            RANGING_STD));
                }
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierError() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2 + inlierError),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateMultipleReadingsPerSourceWithInlierError()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double error1;
            double error2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                } else {
                    error1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                for (int j = 0; j < NUM_READINGS; j++) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        error2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        error2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(error2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + error1 + error2 + inlierError),
                            RANGING_STD));
                }
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
    public void testEstimatePreliminaryNotRefined()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
    public void testEstimateLinearDisabled()
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
            throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
    public void testEstimateLargerPreliminarySubsetSize() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
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

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] sourceQualityScores = new double[numSources];
            double[] fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + error1 + error2),
                        RANGING_STD));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            PROMedSRobustRangingPositionEstimator3D estimator =
                    new PROMedSRobustRangingPositionEstimator3D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint,
                            this);
            estimator.setResultRefined(true);
            estimator.setPreliminarySubsetSize(5);

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

            Point3D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

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
        PROMedSRobustRangingPositionEstimator3D estimator =
                new PROMedSRobustRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RobustRangingPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((PROMedSRobustRangingPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(RobustRangingPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((PROMedSRobustRangingPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            RobustRangingPositionEstimator<Point3D> estimator,
            int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSRobustRangingPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            RobustRangingPositionEstimator<Point3D> estimator,
            float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSRobustRangingPositionEstimator3D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(PROMedSRobustRangingPositionEstimator3D estimator) {
        try {
            estimator.setPreliminarySubsetSize(4);
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
            estimator.setEvenlyDistributeReadings(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setStopThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
