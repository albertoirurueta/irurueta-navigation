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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
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
import org.junit.*;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class LMedSRobustRangingRadioSourceEstimator3DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            LMedSRobustRangingRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)
    private static final double TRANSMITTED_POWER_DBM = -50.0;

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double INLIER_ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    public LMedSRobustRangingRadioSourceEstimator3DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //test empty constructor
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());


        //test constructor with readings
        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0,
                    position));
        }

        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(readings);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with listener
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());


        //test constructor with readings and listener
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(readings, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());


        //test constructor with readings and initial position
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and listener
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition,
                this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());


        //test constructor with readings, initial position and listener
        estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                initialPosition, this);

        //check default values
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LMedSRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getStopThreshold(),
                LMedSRobustRangingRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD,
                0.0);

        //set new value
        estimator.setStopThreshold(0.5);

        //check
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        //set new value
        estimator.setUseReadingPositionCovariances(
                !RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        //check
        assertEquals(estimator.getUseReadingPositionCovariance(),
                !RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getProgressDelta(),
                RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);

        //set new value
        estimator.setProgressDelta(0.5f);

        //check
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getConfidence(),
                RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        //set new value
        estimator.setConfidence(0.5);

        //check
        assertEquals(estimator.getConfidence(), 0.5, 0.0);

        //force IllegalArgumentException
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
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        //set new value
        estimator.setMaxIterations(10);

        //check
        assertEquals(estimator.getMaxIterations(), 10);

        //force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.isResultRefined(),
                RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        //set new value
        estimator.setResultRefined(
                !RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        //check
        assertEquals(estimator.isResultRefined(),
                !RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        //set new value
        estimator.setCovarianceKept(
                !RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        //check
        assertEquals(estimator.isCovarianceKept(),
                !RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testAreValidReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @SuppressWarnings("unchecked")
    @Test
    public void testGetSetReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint,
                    0.0, position));
        }

        LMedSRobustRangingRadioSourceEstimator3D estimator =
                new LMedSRobustRangingRadioSourceEstimator3D();

        //initial value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        //set new value
        estimator.setReadings(readings);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator.isReady());

        //force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setReadings(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getQualityScores());

        //set new value
        estimator.setQualityScores(new double[3]);

        //check
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    //inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                            this);

            estimator.setResultRefined(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertNull(estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;


            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    //inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                            this);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        Math.abs(distance + error), readingsPositions[i]));
            }

            LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                            this);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    LARGE_POSITION_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPositionAndRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    //inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(readings,
                            accessPointPosition, this);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateBeacon() throws LockedException,
            NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    TRANSMITTED_POWER_DBM, FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<Beacon>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    //inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(beacon,
                        distance + error, readingsPositions[i]));
            }

            LMedSRobustRangingRadioSourceEstimator3D<Beacon> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(readings);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            BeaconLocated3D estimatedBeacon =
                    (BeaconLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getTransmittedPower(),
                    TRANSMITTED_POWER_DBM, 0.0);
            assertEquals(estimatedBeacon.getFrequency(), beacon.getFrequency(), 0.0);
            assertEquals(estimatedBeacon.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedBeacon.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithStandardDeviationsAndPositionCovariance()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    //inlier
                    error = 0.0;
                }

                Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i],
                        INLIER_ERROR_STD, positionCovariance));
            }

            LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new LMedSRobustRangingRadioSourceEstimator3D<>(
                            readings, this);
            estimator.setUseReadingPositionCovariances(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        //force NotReadyException
        LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new LMedSRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    @Override
    public void onEstimateEnd(RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    @Override
    public void onEstimateNextIteration(RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator, int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    @Override
    public void onEstimateProgressChange(RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator, float progress) {
        estimateProgressChange++;
        checkLocked((LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private void checkLocked(LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPosition(null);
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
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setResultRefined(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setCovarianceKept(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setReadings(null);
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
