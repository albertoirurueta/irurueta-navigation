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
package com.irurueta.navigation.fingerprinting;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.trilateration.PROMedSRobustTrilateration2DSolver;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolver;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class PROMedSRobustPositionEstimator2DTest implements
        RobustPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            PROMedSRobustPositionEstimator2DTest.class.getName());

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
        //empty constructor
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);


        //constructor with sources
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }
        estimator = new PROMedSRobustPositionEstimator2D(sources);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprints
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator = new PROMedSRobustPositionEstimator2D(fingerprint);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources and fingerprint
        estimator = new PROMedSRobustPositionEstimator2D(sources, fingerprint);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with listener
        estimator = new PROMedSRobustPositionEstimator2D(this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);


        //constructor with sources and listener
        estimator = new PROMedSRobustPositionEstimator2D(sources, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprint and listener
        estimator = new PROMedSRobustPositionEstimator2D(fingerprint, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources, fingerprint and listener
        estimator = new PROMedSRobustPositionEstimator2D(sources, fingerprint,
                this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores
        double[] qualityScores = new double[3];
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D((double[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores and sources
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(null,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    (List<WifiAccessPointLocated2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores and fingerprint
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores, fingerprint);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D((double[])null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores, sources and fingerprint
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                fingerprint);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(null,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores and listener
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D((double[])null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores, sources and listener
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(null,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    (List<WifiAccessPointLocated2D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores, fingerprint and listener
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                fingerprint, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D((double[])null,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with quality scores, sources, fingerprint listener
        estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                fingerprint, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMinRequiredSources(), 3);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
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
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.getNumberOfDimensions(), 2);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRobustPositionEstimator2D(null,
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(new double[1],
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores,
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSRobustTrilateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);

        //set new value
        estimator.setStopThreshold(1.0);

        //check
        assertEquals(estimator.getStopThreshold(), 1.0, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetSources() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertNull(estimator.getSources());

        //set new value
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }

        estimator.setSources(sources);

        //check
        assertSame(estimator.getSources(), sources);

        //force IllegalArgumentException
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
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertNull(estimator.getFingerprint());

        //set new value
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        //check
        assertSame(estimator.getFingerprint(), fingerprint);

        //force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());

        //set new value
        estimator.setRadioSourcePositionCovarianceUsed(true);

        //chekc
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);

        //set new value
        estimator.setFallbackDistanceStandardDeviation(1.0);

        //check
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                1.0, 0.0);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertEquals(estimator.getProgressDelta(),
                RobustTrilaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);

        //set new value
        estimator.setProgressDelta(0.5f);

        //check
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertEquals(estimator.getConfidence(),
                RobustTrilaterationSolver.DEFAULT_CONFIDENCE, 0.0);

        //set new value
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
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                RobustTrilaterationSolver.DEFAULT_MAX_ITERATIONS);

        //set new value
        estimator.setMaxIterations(100);

        //check
        assertEquals(estimator.getMaxIterations(), 100);

        //force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertTrue(estimator.isResultRefined());

        //set new value
        estimator.setResultRefined(false);

        //check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertTrue(estimator.isCovarianceKept());

        //set new value
        estimator.setCovarianceKept(false);

        //check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();

        //check default value
        assertNull(estimator.getQualityScores());

        //set new value
        double[] qualityScores = new double[3];
        estimator.setQualityScores(qualityScores);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);

        //force IllegalArgumentException
        try {
            estimator.setQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);


            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] qualityScores = new double[numSources];
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
                        distance, FREQUENCY, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);


            PROMedSRobustPositionEstimator2D estimator =
                    new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            //check initial state
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

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

        //force NotReadyException
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();
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

        int numValidPosition = 0, numValidCovariance = 0, num = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);


            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] qualityScores = new double[numSources];
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
                        distance, FREQUENCY, pathLossExponent));

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RssiReading<>(accessPoint, rssi + error,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);


            PROMedSRobustPositionEstimator2D estimator =
                    new PROMedSRobustPositionEstimator2D(qualityScores, sources,
                            fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            //check initial state
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

            double positionStd = 0.0;
            boolean hasCovariance = false;
            if (estimator.getInliersData() != null && estimator.getCovariance() != null) {
                assertNotNull(estimator.getInliersData());
                assertNotNull(estimator.getCovariance());

                SingularValueDecomposer decomposer = new SingularValueDecomposer(
                        estimator.getCovariance());
                decomposer.decompose();
                double[] v = decomposer.getSingularValues();
                for (double aV : v) {
                    positionStd += Math.sqrt(aV);
                }
                positionStd /= v.length;

                num++;
                hasCovariance = true;
            }

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance <= LARGE_ABSOLUTE_ERROR) {
                assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;

                if (hasCovariance) {
                    avgValidPositionStd += positionStd;
                    numValidCovariance++;
                }
            } else {
                avgInvalidPositionError += positionDistance;

                if (hasCovariance) {
                    avgInvalidPositionStd += positionStd;
                }
            }

            avgPositionError += positionDistance;

            if (hasCovariance) {
                avgPositionStd += positionStd;
            }
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidCovariance;
        avgInvalidPositionStd /= (num - numValidPosition);
        avgPositionStd /= num;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

        //force NotReadyException
        PROMedSRobustPositionEstimator2D estimator =
                new PROMedSRobustPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RobustPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((PROMedSRobustPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateEnd(RobustPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((PROMedSRobustPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateNextIteration(RobustPositionEstimator<Point2D> estimator,
                                        int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSRobustPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateProgressChange(RobustPositionEstimator<Point2D> estimator,
                                         float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSRobustPositionEstimator2D)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    @SuppressWarnings("all")
    private double receivedPower(double equivalentTransmittedPower,
            double distance, double frequency, double pathLossExponent) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(PROMedSRobustPositionEstimator2D estimator) {
        try {
            estimator.setQualityScores(null);
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
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
