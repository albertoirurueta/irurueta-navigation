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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
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
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

@SuppressWarnings({"Duplicates", "unchecked"})
public class SequentialRobustMixedRadioSourceEstimator3DTest implements
        SequentialRobustMixedRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustMixedRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_READINGS = 100;
    private static final int MAX_READINGS = 500;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double INLIER_ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;
    private static final double LARGE_POWER_ERROR = 0.5;
    private static final double PATH_LOSS_ERROR = 1.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;


    private int estimateStart;
    private int estimateEnd;
    private int estimateProgressChange;

    public SequentialRobustMixedRadioSourceEstimator3DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //test empty constructor
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings
        List<ReadingLocated<Point3D>> readings =
                new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                    0.0, 0.0, position));
        }

        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                readings, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings and initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));

        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings and initial trnasmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(MAX_RSSI,
                this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings, initial position and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with initial position, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial position, initial transmitted power
        //and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings, initial position, initial transmitted power
        //and initial pathloss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition, MAX_RSSI, 1.0);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position, initial transmitted power
        // and initial pathloss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition, MAX_RSSI, 1.0);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with initial position, initial transmitted power,
        // initial pathloss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                initialPosition, MAX_RSSI, 1.0, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial position, initial transmitted power,
        //initial pathloss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(readings,
                initialPosition, MAX_RSSI, 1.0, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    (List<ReadingLocated<Point3D>>)null,
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores
        double[] qualityScores = new double[5];

        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores and readings
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    (List<ReadingLocated<Point3D>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, readings and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    (List<ReadingLocated<Point3D>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores, readings and initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores and initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, readings, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores, initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, readings and initial trnasmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    (List<ReadingLocated<Point3D>>)null,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, readings, initial transmitted power
        // and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    (List<ReadingLocated<Point3D>>)null,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores, readings, initial position and
        // initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with quality scores, initial position and initial
        // transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition, MAX_RSSI);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with quality scores, initial position, initial
        // transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial position, initial transmitted power
        //and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition, MAX_RSSI, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings, initial position, initial transmitted power
        //and initial pathloss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition, MAX_RSSI,
                1.0);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position, initial transmitted power
        // and initial pathloss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition, MAX_RSSI,
                1.0);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with initial position, initial transmitted power,
        // initial pathloss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, initialPosition, MAX_RSSI,
                1.0, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);


        //test constructor with readings, initial position, initial transmitted power,
        //initial pathloss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                qualityScores, readings, initialPosition, MAX_RSSI,
                1.0, this);

        //check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores, null, initialPosition, MAX_RSSI,
                    1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator3D(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point3D>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

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
    public void testGetSetRangingRobustMethod() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);

        //set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        //check
        assertEquals(estimator.getRangingRobustMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testGetSetRssiRobustMethod() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);

        //set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        //check
        assertEquals(estimator.getRssiRobustMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getRangingThreshold());

        //set new value
        estimator.setRangingThreshold(INLIER_ERROR_STD);

        //check
        assertEquals(estimator.getRangingThreshold(), INLIER_ERROR_STD, 0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getRssiThreshold());

        //set new value
        estimator.setRssiThreshold(INLIER_ERROR_STD);

        //check
        assertEquals(estimator.getRssiThreshold(), INLIER_ERROR_STD, 0.0);
    }

    @Test
    public void testGetSetRangingConfidence() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        //set new value
        estimator.setRangingConfidence(0.5);

        //check
        assertEquals(estimator.getRangingConfidence(), 0.5, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setRangingConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setRangingConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRssiConfidence() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        //set new value
        estimator.setRssiConfidence(0.5);

        //check
        assertEquals(estimator.getRssiConfidence(), 0.5, 0.0);

        //force IllegalArgumentException
        try {
            estimator.setRssiConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setRssiConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRangingMaxIterations() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        //set new value
        estimator.setRangingMaxIterations(50);

        //check
        assertEquals(estimator.getRangingMaxIterations(), 50);

        //force IllegalArgumentException
        try {
            estimator.setRangingMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRssiMaxIterations() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        //set new value
        estimator.setRssiMaxIterations(50);

        //check
        assertEquals(estimator.getRssiMaxIterations(), 50);

        //force IllegalArgumentException
        try {
            estimator.setRssiMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        //set new value
        estimator.setResultRefined(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        //check
        assertEquals(estimator.isResultRefined(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        //set new value
        estimator.setCovarianceKept(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        //check
        assertEquals(estimator.isCovarianceKept(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getReadings());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                    0.0, 0.0, position));
        }

        estimator.setReadings(readings);

        //check correctness
        assertSame(estimator.getReadings(), readings);

        //force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setReadings(new ArrayList<>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getQualityScores());

        //set new value
        double[] qualityScores = new double[5];
        estimator.setQualityScores(qualityScores);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);

        //force IllegalArgumentException
        try {
            estimator.setQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(),
                transmittedPowerdBm, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(transmittedPowerdBm), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetInitialTransmittedPower() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getInitialTransmittedPower());

        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
        estimator.setInitialTransmittedPower(transmittedPower);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(),
                transmittedPowerdBm, ABSOLUTE_ERROR);
        assertEquals(estimator.getInitialTransmittedPower(),
                transmittedPower, 0.0);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        Point3D initialPosition = Point3D.create();
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetInitialPathLossExponent() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        //set new value
        estimator.setInitialPathLossExponent(1.0);

        //check
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
    }

    @Test
    public void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);

        //set new value
        estimator.setTransmittedPowerEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);

        //check
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                !RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
    }

    @Test
    public void testIsSetPathLossEstimationEnabled() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.isPathLossEstimationEnabled(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        //set new value
        estimator.setPathLossEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        //check
        assertEquals(estimator.isPathLossEstimationEnabled(),
                !RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        //set new value
        estimator.setUseReadingPositionCovariances(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        //check
        assertEquals(estimator.getUseReadingPositionCovariance(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    public void testAreValidReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 6; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                    0.0, 0.0, position));
        }

        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingAndRssiReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetMinReadings() throws LockedException {
        SequentialRobustMixedRadioSourceEstimator3D estimator =
                new SequentialRobustMixedRadioSourceEstimator3D();

        //check default value
        assertEquals(estimator.getMinReadings(), 5);

        //position only
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        //check
        assertEquals(estimator.getMinReadings(), 4);

        //position and transmitted power
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        //check
        assertEquals(estimator.getMinReadings(), 5);


        //position and pathloss
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 5);


        //position, transmitted power and patloss
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 6);
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
            }

            avgPositionError += positionDistance;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
            }

            avgPowerError += powerError;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= LARGE_POSITION_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        LARGE_POSITION_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= LARGE_POWER_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, LARGE_POWER_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPosition()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPositionAndPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInlierErrorWithInitialPositionAndPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= LARGE_POSITION_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        LARGE_POSITION_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= LARGE_POWER_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, LARGE_POWER_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithPathLossEstimationEnabled()
            throws LockedException, NotReadyException, RobustEstimatorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValidPathLoss = 0,
                numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            if (pathLossVariance <= 0.0) {
                continue;
            }
            assertTrue(pathLossVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);
            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPosition, validPower, validPathLoss;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            double pathLossError = Math.abs(
                    estimator.getEstimatedPathLossExponent() -
                            pathLossExponent);

            if (pathLossError <= PATH_LOSS_ERROR) {
                assertEquals(estimator.getEstimatedPathLossExponent(),
                        pathLossExponent, PATH_LOSS_ERROR);
                validPathLoss = true;
                numValidPathLoss++;

                avgValidPathLossError += pathLossError;
                avgValidPathLossStd += pathLossStd;
            } else {
                validPathLoss = false;

                avgInvalidPathLossError += pathLossError;
                avgInvalidPathLossStd += pathLossStd;
            }

            avgPathLossError += pathLossError;
            avgPathLossStd += pathLossStd;

            if (validPosition && validPower && validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        LOGGER.log(Level.INFO, "Avg. valid path loss error: {0}",
                avgValidPathLossError);
        LOGGER.log(Level.INFO, "Avg. invalid path loss error: {0}",
                avgInvalidPathLossError);
        LOGGER.log(Level.INFO, "Avg. path loss error: {0}",
                avgPathLossError);

        LOGGER.log(Level.INFO, "Valid path loss standard deviation {0}",
                avgValidPathLossStd);
        LOGGER.log(Level.INFO, "Invalid path loss standard deviation {0}",
                avgInvalidPathLossStd);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                avgPathLossStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPathLoss()
            throws LockedException, NotReadyException, RobustEstimatorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPathLossExponent(pathLossExponent);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateBeacon() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D beaconPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);

            BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    transmittedPowerdBm, FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<Beacon>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        beaconPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        beacon.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(beacon,
                        distance, rssi + error,
                        readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<Beacon> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            BeaconWithPowerAndLocated3D estimatedBeacon =
                    (BeaconWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedBeacon.getFrequency(), beacon.getFrequency(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedBeacon.getPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedBeacon.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedBeacon.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedBeacon.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(beaconPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(beaconPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnly() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            boolean validPosition;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            if (validPosition) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnlyWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);

            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            boolean validPosition;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            if (validPosition) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnlyRepeated() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //repeat again so that position covariance matrix is reused
            estimator.estimate();

            //check
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            boolean validPosition;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            if (validPosition) {
                numValid++;
            }

            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionAndPathloss() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPosition, validPathLoss;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double pathLossError = Math.abs(
                    estimator.getEstimatedPathLossExponent() -
                            pathLossExponent);

            if (pathLossError <= PATH_LOSS_ERROR) {
                assertEquals(estimator.getEstimatedPathLossExponent(),
                        pathLossExponent, PATH_LOSS_ERROR);
                validPathLoss = true;
                numValidPathLoss++;

                avgValidPathLossError += pathLossError;
                avgValidPathLossStd += pathLossStd;
            } else {
                validPathLoss = false;

                avgInvalidPathLossError += pathLossError;
                avgInvalidPathLossStd += pathLossStd;
            }

            avgPathLossError += pathLossError;
            avgPathLossStd += pathLossStd;

            if (validPosition && validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid path loss error: {0}",
                avgValidPathLossError);
        LOGGER.log(Level.INFO, "Avg. invalid path loss error: {0}",
                avgInvalidPathLossError);
        LOGGER.log(Level.INFO, "Avg. path loss error: {0}",
                avgPathLossError);

        LOGGER.log(Level.INFO, "Valid path loss standard deviation {0}",
                avgValidPathLossStd);
        LOGGER.log(Level.INFO, "Invalid path loss standard deviation {0}",
                avgInvalidPathLossStd);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                avgPathLossStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionAndPathlossWithInitialValues() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i]));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPosition, validPathLoss;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double pathLossError = Math.abs(
                    estimator.getEstimatedPathLossExponent() -
                            pathLossExponent);

            if (pathLossError <= PATH_LOSS_ERROR) {
                assertEquals(estimator.getEstimatedPathLossExponent(),
                        pathLossExponent, PATH_LOSS_ERROR);
                validPathLoss = true;
                numValidPathLoss++;

                avgValidPathLossError += pathLossError;
                avgValidPathLossStd += pathLossStd;
            } else {
                validPathLoss = false;

                avgInvalidPathLossError += pathLossError;
                avgInvalidPathLossStd += pathLossStd;
            }

            avgPathLossError += pathLossError;
            avgPathLossStd += pathLossStd;

            if (validPosition && validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid path loss error: {0}",
                avgValidPathLossError);
        LOGGER.log(Level.INFO, "Avg. invalid path loss error: {0}",
                avgInvalidPathLossError);
        LOGGER.log(Level.INFO, "Avg. path loss error: {0}",
                avgPathLossError);

        LOGGER.log(Level.INFO, "Valid path loss standard deviation {0}",
                avgValidPathLossStd);
        LOGGER.log(Level.INFO, "Invalid path loss standard deviation {0}",
                avgInvalidPathLossStd);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                avgPathLossStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
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

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD, positionCovariance));
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setUseReadingPositionCovariances(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithMixedRangingAndRssiReadings()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<ReadingLocated<Point3D>> readings =
                    new ArrayList<>();
            double[] qualityScores = new double[3*numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    //outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    //inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0;
                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
                pos++;
            }

            SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator3D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            //check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();
            double powerStd = Math.sqrt(powerVariance);

            boolean validPosition, validPower;
            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                validPosition = true;
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            double powerError = Math.abs(
                    estimator.getEstimatedTransmittedPowerdBm() -
                            transmittedPowerdBm);
            if (powerError <= ABSOLUTE_ERROR) {
                assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                        ABSOLUTE_ERROR);
                assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                        transmittedPowerdBm, ABSOLUTE_ERROR);
                validPower = true;
                numValidPower++;

                avgValidPowerError += powerError;
                avgValidPowerStd += powerStd;
            } else {
                validPower = false;

                avgInvalidPowerError += powerError;
                avgInvalidPowerStd += powerStd;
            }

            avgPowerError += powerError;
            avgPowerStd += powerStd;

            if (validPosition && validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Avg. valid power error: {0} dB",
                avgValidPowerError);
        LOGGER.log(Level.INFO, "Avg. invalid power error: {0} dB",
                avgInvalidPowerError);
        LOGGER.log(Level.INFO, "Avg. power error: {0} dB",
                avgPowerError);

        LOGGER.log(Level.INFO, "Valid power standard deviation {0} dB",
                avgValidPowerStd);
        LOGGER.log(Level.INFO, "Invalid power standard deviation {0} dB",
                avgInvalidPowerStd);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                avgPowerStd);

        //force NotReadyException
        SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    @Override
    public void onEstimateEnd(SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    @Override
    public void onEstimateProgressChange(SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                         float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint>)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private double receivedPower(double equivalentTransmittedPower, double distance, double frequency,
                                 double pathLossExponent) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(SequentialRobustMixedRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingRobustMethod(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiRobustMethod(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRangingMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRssiMaxIterations(10);
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
            estimator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialTransmittedPowerdBm(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialTransmittedPower(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPathLossExponent(2.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setTransmittedPowerEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPathLossEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setUseReadingPositionCovariances(true);
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
