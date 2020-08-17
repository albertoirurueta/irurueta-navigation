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
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconIdentifier;
import com.irurueta.navigation.indoor.BeaconLocated3D;
import com.irurueta.navigation.indoor.IndoorException;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.indoor.RangingReadingLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class RangingRadioSourceEstimator3DTest implements
        RangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            RangingRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)
    private static final double TRANSMITTED_POWER_DBM = -50.0;

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ERROR_STD = 0.2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public RangingRadioSourceEstimator3DTest() {
    }

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test empty constructor
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // test constructor with readings
        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0,
                    position));
        }

        estimator = new RangingRadioSourceEstimator3D<>(readings);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener
        estimator = new RangingRadioSourceEstimator3D<>(this);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        // test constructor with readings and listener
        estimator = new RangingRadioSourceEstimator3D<>(readings, this);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with initial position
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RangingRadioSourceEstimator3D<>(initialPosition);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        // test constructor with readings and initial position
        estimator = new RangingRadioSourceEstimator3D<>(readings, initialPosition);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with initial position and listener
        estimator = new RangingRadioSourceEstimator3D<>(initialPosition, this);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        // test constructor with readings, initial position and listener
        estimator = new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                this);

        // check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(estimator.isHomogeneousLinearSolverUsed(),
                RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testIsSetNonLinearSolverEnabled() throws LockedException {
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default value
        assertTrue(estimator.isNonLinearSolverEnabled());

        // set new value
        estimator.setNonLinearSolverEnabled(false);

        // check
        assertFalse(estimator.isNonLinearSolverEnabled());

        // set new value
        estimator.setNonLinearSolverEnabled(true);

        // check
        assertTrue(estimator.isNonLinearSolverEnabled());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default value
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // set new value
        estimator.setUseReadingPositionCovariances(
                !RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(estimator.getUseReadingPositionCovariance(),
                !RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    public void testAreValidReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint,
                    0.0, position));
        }

        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint,
                    0.0, position));
        }

        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // initial value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        // set new value
        estimator.setReadings(readings);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator.isReady());

        // force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateWithoutInitialPositionAndWithoutError()
            throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInitialPositionAndWithoutError()
            throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, accessPointPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>(new InhomogeneousPoint3D());

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithoutInitialPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithInitialPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, accessPointPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithInitialPositionWithError()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomZ() + randomizer.nextDouble(MIN_POS, MAX_POS));
            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithInitialPositionWithPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomZ() + randomizer.nextDouble(MIN_POS, MAX_POS));
            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateBeacon() throws LockedException, NotReadyException,
            AlgebraException, RadioSourceEstimationException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));

            final BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    TRANSMITTED_POWER_DBM, FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<Beacon>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(beacon, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<Beacon> estimator =
                    new RangingRadioSourceEstimator3D<>(readings);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final BeaconLocated3D estimatedBeacon =
                    (BeaconLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getFrequency(), beacon.getFrequency(), 0.0);
            assertEquals(estimatedBeacon.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedBeacon.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final RangingRadioSourceEstimator3D<Beacon> estimator =
                new RangingRadioSourceEstimator3D<>();

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithNonLinearSolverDisabledAndWithoutInitialPosition()
            throws LockedException, NotReadyException {

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertNull(estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithNonLinearSolverDisabledAndWithInitialPosition()
            throws LockedException, NotReadyException {

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, accessPointPosition, this);
            estimator.setNonLinearSolverEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);


        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithRangingStandardDeviations()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i], ERROR_STD));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithReadingPositionCovariances()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomZ() + errorRandomizer.nextDouble());

                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance, readingsPositions[i], positionCovariance));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setUseReadingPositionCovariances(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithRangingStandardDeviationsAndReadingPositionCovariances()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomZ() + errorRandomizer.nextDouble());

                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i], ERROR_STD,
                        positionCovariance));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setUseReadingPositionCovariances(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateWithPositionCovariancesDisabled()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);
                final double error = Math.abs(errorRandomizer.nextDouble());

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomZ() + errorRandomizer.nextDouble());

                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i], ERROR_STD,
                        positionCovariance));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setUseReadingPositionCovariances(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateInitialPositionHomogeneousSolver()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setNonLinearSolverEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateInitialPositionInhomogeneousSolver()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setNonLinearSolverEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateNonLinearDisabledAndHomogeneousSolver()
            throws LockedException, NotReadyException {

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);
            estimator.setHomogeneousLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);


        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Test
    public void testEstimateNonLinearDisableAndInhomogeneousSolver()
            throws LockedException, NotReadyException {

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            final RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);


        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
    }

    @Override
    public void onEstimateStart(
            final RangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(
            final RangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(
            final RangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        try {
            estimator.setNonLinearSolverEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHomogeneousLinearSolverUsed(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseReadingPositionCovariances(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReadings(null);
            fail("LockedExeption expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
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
