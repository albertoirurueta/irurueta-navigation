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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.trilateration.LMedSRobustTrilateration2DSolver;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolver;
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

public class LMedSRobustRssiPositionEstimator2DTest implements
        RobustRssiPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            LMedSRobustRssiPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 100;
    private static final int MAX_SOURCES = 500;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 0.5;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private static final double INLIER_ERROR_STD = 0.1;

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }
        estimator = new LMedSRobustRssiPositionEstimator2D(sources);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprints
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator = new LMedSRobustRssiPositionEstimator2D(fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources and fingerprint
        estimator = new LMedSRobustRssiPositionEstimator2D(sources, fingerprint);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with listener
        estimator = new LMedSRobustRssiPositionEstimator2D(this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());


        // constructor with sources and listener
        estimator = new LMedSRobustRssiPositionEstimator2D(sources, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint and listener
        estimator = new LMedSRobustRssiPositionEstimator2D(fingerprint, this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
                this);

        // check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                RobustTrilaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustTrilaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(null, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRssiPositionEstimator2D(sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
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
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        LMedSRobustRssiPositionEstimator2D solver =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        Point2D p = Point2D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(solver.getInitialPosition(), p);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);

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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isLinearSolverUsed());

        // set new value
        estimator.setLinearSolverUsed(false);

        // check
        assertFalse(estimator.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.isPreliminarySolutionRefined());

        // set new value
        estimator.setPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetSourceQualityScores() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        double[] qualityScores = new double[1];
        estimator.setSourceQualityScores(qualityScores);

        // check
        assertNull(estimator.getSourceQualityScores());
    }

    @Test
    public void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        double[] qualityScores = new double[1];
        estimator.setFingerprintReadingsQualityScores(qualityScores);

        // check
        assertNull(estimator.getFingerprintReadingsQualityScores());
    }

    @Test
    public void testGetSetEvenlyDistributeReadings() throws LockedException {
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();

        // check default value
        assertTrue(estimator.getEvenlyDistributeReadings());

        // set new value
        estimator.setEvenlyDistributeReadings(false);

        // check
        assertFalse(estimator.getEvenlyDistributeReadings());
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange > 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);


            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);


            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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
        LMedSRobustRssiPositionEstimator2D estimator =
                new LMedSRobustRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined()
            throws NonSymmetricPositiveDefiniteMatrixException, LockedException,
            NotReadyException, RobustEstimatorException {
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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
    public void testEstimateLinearSolverUsedInhomogeneousPreliminaryRefined()
            throws NonSymmetricPositiveDefiniteMatrixException, LockedException,
            NotReadyException, RobustEstimatorException {
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double error;
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
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
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            LMedSRobustRssiPositionEstimator2D estimator =
                    new LMedSRobustRssiPositionEstimator2D(sources, fingerprint,
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

            Point2D p = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
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

    @Override
    public void onEstimateStart(RobustRssiPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((LMedSRobustRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(RobustRssiPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((LMedSRobustRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateNextIteration(RobustRssiPositionEstimator<Point2D> estimator,
                                        int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSRobustRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateProgressChange(RobustRssiPositionEstimator<Point2D> estimator,
                                         float progress) {
        estimateProgressChange++;
        checkLocked((LMedSRobustRssiPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private double receivedPower(double equivalentTransmittedPower,
                                 double distance, double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * LMedSRobustRssiPositionEstimator2DTest.FREQUENCY),
                pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(LMedSRobustRssiPositionEstimator2D estimator) {
        try {
            estimator.setEvenlyDistributeReadings(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(100);
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
            estimator.setLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setStopThreshold(1.0);
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
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
