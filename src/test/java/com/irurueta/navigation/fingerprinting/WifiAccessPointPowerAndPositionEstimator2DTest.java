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
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;
import static com.irurueta.navigation.fingerprinting.Utils.*;

public class WifiAccessPointPowerAndPositionEstimator2DTest implements
        WifiAccessPointPowerAndPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            WifiAccessPointPowerAndPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;
    private static final double LARGE_POWER_ERROR = 0.5;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public WifiAccessPointPowerAndPositionEstimator2DTest() { }

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings
        List<WifiReadingLocated2D> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 3; i++) {
            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new WifiReadingLocated2D(accessPoint, 0.0, position));
        }

        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    (List<WifiReadingLocated2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    (List<WifiReadingLocated2D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and initial position
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(initialPosition,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial position and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                initialPosition, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    null, initialPosition,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), initialPosition,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and initial transmitted power
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    (List<WifiReadingLocated2D>)null, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(MAX_RSSI,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial transmitted power and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    (List<WifiReadingLocated2D>)null, MAX_RSSI,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), MAX_RSSI,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings, initial position and
        //initial transmitted power
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                initialPosition, MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), initialPosition,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and initial transmitted power
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                initialPosition, MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with initial position, initial transmitted power and
        //listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                initialPosition, MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial position, initial
        //transmitted power and listener
        estimator = new WifiAccessPointPowerAndPositionEstimator2D(readings,
                initialPosition, MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 3);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getEstimatedTransmittedPowerVariance(),
                0.0, 0.0);
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    null, initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WifiAccessPointPowerAndPositionEstimator2D(
                    new ArrayList<WifiReadingLocated2D>(), initialPosition,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        //check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double value = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(value);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), value, 0.0);
    }

    @Test
    public void testGetSetInitialTransmittedPower() throws LockedException {
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        //check default value
        assertNull(estimator.getInitialTransmittedPower());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double value = Utils.dBmToPower(
                randomizer.nextDouble(MIN_RSSI, MAX_RSSI));
        estimator.setInitialTransmittedPower(value);

        //check
        assertEquals(estimator.getInitialTransmittedPower(), value,
                ABSOLUTE_ERROR);

        //force IllegalArgumentException
        try {
            estimator.setInitialTransmittedPower(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        //set null value
        estimator.setInitialTransmittedPower(null);

        //check
        assertNull(estimator.getInitialTransmittedPower());
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testAreValidReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<WifiReadingLocated2D> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 3; i++) {
            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new WifiReadingLocated2D(accessPoint, 0.0, position));
        }

        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<WifiReadingLocated<Point2D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<WifiReadingLocated2D> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 3; i++) {
            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new WifiReadingLocated2D(accessPoint, 0.0,
                    position));
        }

        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

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
    }

    @Test
    public void testGetSetListener() throws LockedException {
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithoutError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
            avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency()));

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i]));
            }

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionWithoutError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency()));

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialTransmittedPowerWithoutError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency()));

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialTransmittedPowerdBm, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndInitialTransmittedPowerWithoutError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency()));

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialPosition, initialTransmittedPowerdBm,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency())) + error;

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndWithError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency())) + error;

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency())) + error;

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialTransmittedPowerdBm, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, FingerprintingException, AlgebraException {

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point2D[] readingsPositions = new Point2D[numReadings];
            List<WifiReadingLocated2D> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency())) + error;

                readings.add(new WifiReadingLocated2D(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            WifiAccessPointPowerAndPositionEstimator2D estimator =
                    new WifiAccessPointPowerAndPositionEstimator2D(readings,
                            initialPosition, initialTransmittedPowerdBm, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    estimator.getEstimatedPositionCovariance());
            decomposer.decompose();
            double[] v = decomposer.getSingularValues();
            double positionStd = 0.0;
            for (double aV : v) {
                positionStd += Math.sqrt(aV);
            }
            positionStd /= v.length;
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
            } else {
                validPosition = false;

                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;

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

            if (validPosition && validPower) {
                numValid++;
            }

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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

        LOGGER.log(Level.INFO, "Valid position standard deviation {0} meters",
                avgValidPositionStd);
        LOGGER.log(Level.INFO, "Invalid position standard deviation {0} meters",
                avgInvalidPositionStd);
        LOGGER.log(Level.INFO, "Position standard deviation {0} meters",
                avgPositionStd);

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
        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateFingerprintingException() throws LockedException,
            NotReadyException {

        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

        int numReadings = randomizer.nextInt(
                MIN_READINGS, MAX_READINGS);
        Point2D[] readingsPositions = new Point2D[numReadings];
        List<WifiReadingLocated2D> readings = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            readingsPositions[i] = new InhomogeneousPoint2D(
                    0.0, 0.0);

            readings.add(new WifiReadingLocated2D(accessPoint, 0.0,
                    readingsPositions[i]));
        }

        WifiAccessPointPowerAndPositionEstimator2D estimator =
                new WifiAccessPointPowerAndPositionEstimator2D(readings,
                        this);

        reset();
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);

        try {
            estimator.estimate();
            fail("FingerprintingException expected but not thrown");
        } catch (FingerprintingException ignore) { }
    }

    @Override
    public void onEstimateStart(WifiAccessPointPowerAndPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((WifiAccessPointPowerAndPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateEnd(WifiAccessPointPowerAndPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((WifiAccessPointPowerAndPositionEstimator2D)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private double receivedPower(double equivalentTransmittedPower, double distance, double frequency) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), 2.0);
        return equivalentTransmittedPower * k /
                (distance * distance);
    }

    private void checkLocked(WifiAccessPointPowerAndPositionEstimator2D estimator) {
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