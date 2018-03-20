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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BeaconTest {

    public BeaconTest() { }

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
        //test empty constructor
        Beacon b = new Beacon();

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
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);


        //test constructor with identifiers and transmitted power
        List<BeaconIdentifier> identifiers = new ArrayList<>();
        b = new Beacon(identifiers, -50.0);

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
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new Beacon(null, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with all data
        b = new Beacon(identifiers, -50.0, "address", 1, 2,
                3, "name");

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
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new Beacon(null, -50.0, "address", 1,
                    2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);


        //test constructor with identifiers, transmitted power and frequency
        b = new Beacon(identifiers, -50.0, 5.0e9);

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
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new Beacon(null, -50.0, 5.0e9);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new Beacon(identifiers, -50.0, -5.0e9);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);

        //test constructor with all data and frequency
        b = new Beacon(identifiers, -50.0, 5.0e9, "address", 1, 2,
                3, "name");

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
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        //force IllegalArgumentException
        b = null;
        try {
            b = new Beacon(null, -50.0, 5.0e9, "address",
                    1, 2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            b = new Beacon(identifiers, -50.0, -5.0e9, "address",
                    1, 2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(b);
    }

    @Test
    public void testGetIdentifier() {
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

        List<BeaconIdentifier> identifiers = new ArrayList<>();
        identifiers.add(id1);
        identifiers.add(id2);
        identifiers.add(id3);

        Beacon b = new Beacon(identifiers, -60.0);

        //check
        assertEquals(b.getIdentifier(0), id1);
        assertEquals(b.getIdentifier(1), id2);
        assertEquals(b.getIdentifier(2), id3);

        assertEquals(b.getId1(), id1);
        assertEquals(b.getId2(), id2);
        assertEquals(b.getId3(), id3);

        assertNull(b.getIdentifier(-1));
        assertNull(b.getIdentifier(identifiers.size()));
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


        Beacon b1 = new Beacon(identifiers1, -60.0);
        Beacon b2 = new Beacon(identifiers1, -50.0);
        Beacon b3 = new Beacon(identifiers2, -60.0);

        //check
        assertEquals(b1, b1);
        assertEquals(b1, b2);
        assertNotEquals(b1, b3);

        assertFalse(b1.equals(new Object()));

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }
}
