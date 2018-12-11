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
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static com.irurueta.navigation.indoor.Utils.dBmToPower;
import static com.irurueta.navigation.indoor.Utils.powerTodBm;
import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class RssiRadioSourceEstimator3DTest implements
        RssiRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            RssiRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double ERROR_STD = 0.2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;
    private static final double LARGE_POWER_ERROR = 0.5;
    private static final double PATH_LOSS_ERROR = 1.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 5;

    private int estimateStart;
    private int estimateEnd;

    public RssiRadioSourceEstimator3DTest() { }

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings
        List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        estimator = new RssiRadioSourceEstimator3D<>(readings);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    (List<RssiReadingLocated3D<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with listener
        estimator = new RssiRadioSourceEstimator3D<>(this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and listener
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    (List<RssiReadingLocated3D<WifiAccessPoint>>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RssiRadioSourceEstimator3D<>(initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and initial position
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and listener
        estimator = new RssiRadioSourceEstimator3D<>(initialPosition,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial position and listener
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power
        estimator = new RssiRadioSourceEstimator3D<>(MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        Assert.assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings and initial transmitted power
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    (List<RssiReadingLocated3D<WifiAccessPoint>>)null, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial transmitted power and listener
        estimator = new RssiRadioSourceEstimator3D<>(MAX_RSSI,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial transmitted power and listener
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    (List<RssiReadingLocated3D<WifiAccessPoint>>)null, MAX_RSSI,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), MAX_RSSI,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with readings, initial position and
        //initial transmitted power
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and initial transmitted power
        estimator = new RssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with initial position, initial transmitted power and
        //listener
        estimator = new RssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial position, initial
        //transmitted power and listener
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        //test constructor with readings, initial position, initial
        //transmitted power and initial path loss exponent
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition, MAX_RSSI,
                    MIN_PATH_LOSS_EXPONENT);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition,
                    MAX_RSSI, MIN_PATH_LOSS_EXPONENT);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and initial transmitted power
        estimator = new RssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with initial position, initial transmitted power and
        //listener
        estimator = new RssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);


        //test constructor with readings, initial position, initial
        //transmitted power and listener
        estimator = new RssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 5);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator.isPositionEstimationEnabled());
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    null, initialPosition, MAX_RSSI,
                    MIN_PATH_LOSS_EXPONENT, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RssiRadioSourceEstimator3D<>(
                    new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>(), initialPosition,
                    MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetMinReadings() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getMinReadings(), 5);

        //position only
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        //check
        assertEquals(estimator.getMinReadings(), 4);


        //transmitted power only
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        //check
        assertEquals(estimator.getMinReadings(), 2);


        //pathloss only
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 2);


        //position and transmitted power
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        //check
        assertEquals(estimator.getMinReadings(), 5);


        //position and pathloss
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 5);


        //transmitted power and pathloss
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 3);


        //position, transmitted power and patloss
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertEquals(estimator.getMinReadings(), 6);
    }

    @Test
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetInitialPathLossExponent() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double value = randomizer.nextDouble(
                MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        estimator.setInitialPathLossExponent(value);

        //check
        assertEquals(estimator.getInitialPathLossExponent(),
                value, 0.0);
    }

    @Test
    public void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());

        //set new value
        estimator.setTransmittedPowerEstimationEnabled(false);

        //check
        assertFalse(estimator.isTransmittedPowerEstimationEnabled());
    }

    @Test
    public void testIsSetPositionEstimationEnabled() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertTrue(estimator.isPositionEstimationEnabled());

        //set new value
        estimator.setPositionEstimationEnabled(false);

        //check
        assertFalse(estimator.isPositionEstimationEnabled());
    }

    @Test
    public void testIsSetPathLossEstimationEnabled() throws LockedException {
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertFalse(estimator.isPathLossEstimationEnabled());

        //set new value
        estimator.setPathLossEstimationEnabled(true);

        //check
        assertTrue(estimator.isPathLossEstimationEnabled());
    }

    @Test
    public void testAreValidReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RssiReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0,
                    position));
        }

        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);
        estimator.setPathLossEstimationEnabled(false);
        estimator.setInitialPathLossExponent(MAX_PATH_LOSS_EXPONENT);

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialPosition, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialTransmittedPowerWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialTransmittedPowerdBm, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndInitialTransmittedPowerWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);
            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialPosition, initialTransmittedPowerdBm,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
        for (int t = 0; t < 10*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + errorRandomizer.nextDouble());

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialPosition, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
        for (int t = 0; t < 2 * TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialTransmittedPowerdBm, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
        for (int t = 0; t < 2 * TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + errorRandomizer.nextDouble());
            double initialTransmittedPowerdBm = transmittedPowerdBm +
                    errorRandomizer.nextDouble();

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            initialPosition, initialTransmittedPowerdBm, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabled()
            throws LockedException, NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabledWithInitialValues()
            throws LockedException, NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPathLoss()
            throws LockedException, NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
                    pathLossExponent, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), pathLossExponent, 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateBeacon() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D beaconPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);

            BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    transmittedPowerdBm, FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<Beacon>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        beaconPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        beacon.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RssiReadingLocated3D<>(beacon, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<Beacon> estimator =
                    new RssiRadioSourceEstimator3D<>(readings);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

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
                    estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
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
        RssiRadioSourceEstimator3D<Beacon> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnly() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnlyWithInitialPosition() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionOnlyRepeated() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        int numValidPosition = 0, numValid = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //repet again so that position covariance matrix is reused
            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateTransmittedPowerOnly() throws LockedException,
            NotReadyException, IndoorException {
        int numValidPower = 0, numValid = 0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(),
                    pathLossExponent, 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);

            double powerStd = Math.sqrt(powerVariance);

            boolean validPower;

            double powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
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

            if (validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateTransmittedPowerOnlyWithInitialTransmittedPower() throws LockedException,
            NotReadyException, IndoorException {
        int numValidPower = 0, numValid = 0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(),
                    pathLossExponent, 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);

            double powerStd = Math.sqrt(powerVariance);

            boolean validPower;

            double powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
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

            if (validPower) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePathlossOnly() throws LockedException,
            NotReadyException, IndoorException {
        int numValidPathLoss = 0, numValid = 0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);

            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPathLoss;

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

            if (validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePathlossOnlyWithInitialPathloss() throws LockedException,
            NotReadyException, IndoorException {
        int numValidPathLoss = 0, numValid = 0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);

            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPathLoss;

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

            if (validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionAndPathloss() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimatePositionAndPathlossWithInitialValues() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i], ERROR_STD));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
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

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateTransmittedPowerAndPathLoss() throws LockedException,
            NotReadyException, IndoorException {

        int numValidPower = 0, numValidPathLoss = 0, numValid = 0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            double powerStd = Math.sqrt(powerVariance);
            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPower, validPathLoss;
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

            if (validPower && validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateTransmittedPowerAndPathLossWithInitialValues() throws LockedException,
            NotReadyException, IndoorException {

        int numValidPower = 0, numValidPathLoss = 0, numValid = 0;
        double avgPowerError = 0.0, avgValidPowerError = 0.0,
                avgInvalidPowerError = 0.0;
        double avgPathLossError = 0.0, avgValidPathLossError = 0.0,
                avgInvalidPathLossError = 0.0;
        double avgPowerStd = 0.0, avgValidPowerStd = 0.0,
                avgInvalidPowerStd = 0.0;
        double avgPathLossStd = 0.0, avgValidPathLossStd = 0.0,
                avgInvalidPathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = dBmToPower(transmittedPowerdBm);
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double rssi = powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi,
                        readingsPositions[i]));
            }

            RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RssiRadioSourceEstimator3D<>(readings,
                            this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D)estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPosition(), accessPointPosition);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());

            double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            double powerStd = Math.sqrt(powerVariance);
            double pathLossStd = Math.sqrt(pathLossVariance);

            boolean validPower, validPathLoss;
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

            if (validPower && validPathLoss) {
                numValid++;
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);
        assertTrue(numValid > 0);

        avgValidPowerError /= numValidPower;
        avgInvalidPowerError /= (TIMES - numValidPower);
        avgPowerError /= TIMES;

        avgValidPathLossError /= numValidPathLoss;
        avgInvalidPathLossError /= (TIMES - numValidPathLoss);
        avgPathLossError /= TIMES;

        avgValidPowerStd /= numValidPower;
        avgInvalidPowerStd /= (TIMES - numValidPower);
        avgPowerStd /= TIMES;

        avgValidPathLossStd /= numValidPathLoss;
        avgInvalidPathLossStd /= (TIMES - numValidPathLoss);
        avgPathLossStd /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double)numValidPower / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid path loss: {0} %",
                (double)numValidPathLoss / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage all valid: {0} %",
                (double)numValid / (double)TIMES * 100.0);

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
        RssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator2D<>();
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
        Point3D[] readingsPositions = new Point3D[numReadings];
        List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            readingsPositions[i] = new InhomogeneousPoint3D(
                    0.0, 0.0, 0.0);

            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0,
                    readingsPositions[i]));
        }

        RssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RssiRadioSourceEstimator3D<>(readings,
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
            fail("IndoorException expected but not thrown");
        } catch (IndoorException ignore) { }
    }

    @Override
    public void onEstimateStart(RssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(RssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

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

    private void checkLocked(RssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
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
