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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BeaconWithPowerAndLocated2DTest {

    public BeaconWithPowerAndLocated2DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws AlgebraException {
        //test empty constructor
        BeaconWithPowerAndLocated2D b = new BeaconWithPowerAndLocated2D();

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertNull(b.getIdentifiers());
        assertEquals(b.getTransmittedPower(), 0.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertNull(b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);


        //test constructor with identifiers, transmitted power and position
        List<BeaconIdentifier> identifiers = new ArrayList<>();
        InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power, transmitted
        //power standard deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                null, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, transmitted power standard deviation
        //and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", null,
                position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power, position and
        //position covariance
        Matrix cov = new Matrix(2, 2);
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                position, null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    "address", 1,
                    2, 3, "name",
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power, transmitted
        //power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                null, position, null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    Double.valueOf(1.0), position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, transmitted power standard deviation,
        //position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", null,
                position, null);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    "address", 1, 2,
                    3, "name", Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position and frequency
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power, transmitted
        //power standard deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                Double.valueOf(1.0), position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                null, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, transmitted power standard deviation
        //and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", null,
                position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power, transmitted
        //power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, Double.valueOf(1.0), position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, null, position,
                null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, Double.valueOf(1.0), position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, transmitted power standard deviation,
        //position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", null,
                position, null);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, path loss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                1.6, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, pathloss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, std dev, path loss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                null, 1.6,
                position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power, std, position and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx, power, position, position covariance
        // and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                1.6, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                1.6, position, null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position, position covariance and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    "address", 1,
                    2, 3, "name",
                    1.6, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance and path loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                null, 1.6,
                position, null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    Double.valueOf(1.0), 1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power std deviation, position,
        //position covariance and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null, position,
                null);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    "address", 1, 2,
                    3, "name", 1.6,
                    1.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position, frequency and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, tx power std deviation,
        //position and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6, Double.valueOf(1.0),
                position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6,
                null, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    Double.valueOf(1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(-1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power std deviation, position and
        //pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                null, position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position, position covariance and path loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, tx power std deviation,
        //position, position covariance and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6, Double.valueOf(1.0),
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6,
                null, position,
                null);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    Double.valueOf(1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(-1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, 1.6, Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power std deviation, position,
        //position covariance and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                1.0, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                null, position,
                null);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, path loss, path loss
        // std deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6,
                Double.valueOf(0.1), position);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(),
                0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, Double.valueOf(0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(-0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, path loss, path loss std deviation and
        //position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, std dev, path loss,
        //path loss std deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, Double.valueOf(0.1),
                position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                null, 1.6,
                null, position, cov);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, Double.valueOf(0.1), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), null,
                    cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position,
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power, stds, position and pathloss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null,
                null, position, cov);

        //check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx, power, position, path loss and
        //pathloss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6,
                1.0,
                0.1, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentexception
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, position, path loss and pathloss std
        // deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    -5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, tx power, tx power std deviation,
        //position, position covariance, pathloss and pathloss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, 1.6,
                1.0,
                0.1, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    -5.0e9, 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, 1.6,
                    -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data, tx power std deviation, position,
        //position covariance, pathloss and pathloss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                5.0e9, "address", 1,
                2, 3, "name",
                1.6, 1.0,
                0.1, position, cov);

        //check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated2D(null, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    -5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new BeaconWithPowerAndLocated2D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);
    }

    @Test
    public void testEquals() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long value1 = randomizer.nextLong();
        BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1,
                Long.SIZE / Byte.SIZE);

        long value2 = randomizer.nextLong();
        BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2,
                Long.SIZE / Byte.SIZE);

        long value3 = randomizer.nextLong();
        BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3,
                Long.SIZE / Byte.SIZE);

        List<BeaconIdentifier> identifiers1 = new ArrayList<>();
        identifiers1.add(id1);
        identifiers1.add(id2);
        identifiers1.add(id3);

        long value4 = value1 + 1;
        BeaconIdentifier id4 = BeaconIdentifier.fromLong(value4,
                Long.SIZE / Byte.SIZE);

        long value5 = value2 + 1;
        BeaconIdentifier id5 = BeaconIdentifier.fromLong(value5,
                Long.SIZE / Byte.SIZE);

        long value6 = value3 + 1;
        BeaconIdentifier id6 = BeaconIdentifier.fromLong(value6,
                Long.SIZE / Byte.SIZE);

        List<BeaconIdentifier> identifiers2 = new ArrayList<>();
        identifiers2.add(id4);
        identifiers2.add(id5);
        identifiers2.add(id6);

        InhomogeneousPoint2D position = new InhomogeneousPoint2D();

        BeaconWithPowerAndLocated2D b1 = new BeaconWithPowerAndLocated2D(
                identifiers1, -60.0, position);
        BeaconWithPowerAndLocated2D b2 = new BeaconWithPowerAndLocated2D(
                identifiers1, -50.0, position);
        BeaconWithPowerAndLocated2D b3 = new BeaconWithPowerAndLocated2D(
                identifiers2, -60.0, position);

        //check
        assertEquals(b1, b1);
        assertEquals(b1, b2);
        assertNotEquals(b1, b3);

        assertFalse(b1.equals(new Object()));

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }
}
