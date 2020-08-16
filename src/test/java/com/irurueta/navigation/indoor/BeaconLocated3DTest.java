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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BeaconLocated3DTest {

    public BeaconLocated3DTest() {
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
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        BeaconLocated3D b = new BeaconLocated3D();

        // check default values
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
        assertNull(b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);


        // test constructor with identifiers, transmitted power and position
        final List<BeaconIdentifier> identifiers = new ArrayList<>();
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        b = new BeaconLocated3D(identifiers, -50.0, position);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with all data and position
        b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                2, 3, "name", position);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, "address", 1,
                    2, 3, "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                    2, 3, "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with identifiers, transmitted power, position and
        // position covariance
        final Matrix cov = new Matrix(3, 3);
        b = new BeaconLocated3D(identifiers, -50.0, position, cov);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconLocated3D(identifiers, -50.0, position,
                null);

        // check
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with all data, position and position covariance
        b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                2, 3, "name", position, cov);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                2, 3, "name", position, null);

        // check
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, "address", 1,
                    2, 3, "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                    2, 3, "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, "address", 1,
                    2, 3, "name", position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with identifiers, transmitted power, position and frequency
        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9, position);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, 5.0e9, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, -5.0e9, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with all data and position
        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", position);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, 5.0e9,
                    "address", 1,
                    2, 3, "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, -5.0e9,
                    "address", 1, 2,
                    3, "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                    "address", 1,
                    2, 3, "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with identifiers, transmitted power, position and
        // position covariance
        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9, position, cov);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9, position,
                null);

        // check
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, 5.0e9,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, -5.0e9,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);


        // test constructor with all data, position and position covariance
        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", position, cov);

        // check default values
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
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", position, null);

        // check
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
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconLocated3D(null, -50.0, 5.0e9,
                    "address", 1,
                    2, 3, "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, -5.0e9,
                    "address", 1, 2,
                    3, "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                    "address", 1,
                    2, 3, "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconLocated3D(identifiers, -50.0, 5.0e9,
                    "address", 1,
                    2, 3, "name", position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long value1 = randomizer.nextLong();
        final BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1,
                Long.SIZE / Byte.SIZE);

        final long value2 = randomizer.nextLong();
        final BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2,
                Long.SIZE / Byte.SIZE);

        final long value3 = randomizer.nextLong();
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3,
                Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers1 = new ArrayList<>();
        identifiers1.add(id1);
        identifiers1.add(id2);
        identifiers1.add(id3);

        final long value4 = value1 + 1;
        final BeaconIdentifier id4 = BeaconIdentifier.fromLong(value4,
                Long.SIZE / Byte.SIZE);

        final long value5 = value2 + 1;
        final BeaconIdentifier id5 = BeaconIdentifier.fromLong(value5,
                Long.SIZE / Byte.SIZE);

        final long value6 = value3 + 1;
        final BeaconIdentifier id6 = BeaconIdentifier.fromLong(value6,
                Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers2 = new ArrayList<>();
        identifiers2.add(id4);
        identifiers2.add(id5);
        identifiers2.add(id6);

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();

        final BeaconLocated3D b1 = new BeaconLocated3D(identifiers1,
                -60.0, position);
        final BeaconLocated3D b2 = new BeaconLocated3D(identifiers1,
                -50.0, position);
        final BeaconLocated3D b3 = new BeaconLocated3D(identifiers2,
                -60.0, position);

        // check
        assertEquals(b1, b1);
        assertEquals(b1, b2);
        assertNotEquals(b1, b3);

        assertNotEquals(b1, new Object());

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }
}
