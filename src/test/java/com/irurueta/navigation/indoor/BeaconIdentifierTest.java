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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Random;
import java.util.UUID;

import static org.junit.Assert.*;

public class BeaconIdentifierTest {

    public BeaconIdentifierTest() {
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
        // test empty constructor
        BeaconIdentifier id = new BeaconIdentifier();

        // check
        assertNotNull(id.toString());
        assertEquals(id.toInt(), 0);
        assertNull(id.toByteArrayOfSpecifiedEndianness(true));
        assertEquals(id.getByteCount(), 0);
        assertNull(id.toHexString());
        assertNull(id.toUuid());
        assertNull(id.toByteArray());
        assertEquals(id.hashCode(), 0);

        // test constructor from byte array
        final Random r = new Random();

        // length 2
        byte[] value = new byte[2];
        r.nextBytes(value);

        id = new BeaconIdentifier(value);

        // check
        assertEquals(id.toString(), Integer.toString(id.toInt()));
        assertTrue(id.toInt() > 0);
        assertArrayEquals(id.toByteArrayOfSpecifiedEndianness(true),
                value);
        final byte[] reversedValue = id.toByteArrayOfSpecifiedEndianness(false);
        for (int i = 0; i < value.length; i++) {
            assertEquals(value[i], reversedValue[value.length - 1 - i]);
        }
        assertEquals(id.getByteCount(), value.length);

        assertNotNull(id.toHexString());
        BeaconIdentifier id2 = BeaconIdentifier.parse(id.toHexString());
        assertArrayEquals(id.toByteArray(), id2.toByteArray());
        assertEquals(id, id2);
        assertEquals(id.hashCode(), id2.hashCode());

        try {
            id.toUuid();
            fail("UnsupportedOperationException expected but not thrown");
        } catch (final UnsupportedOperationException ignore) {
        }

        //length 16
        value = new byte[16];
        r.nextBytes(value);
        id = new BeaconIdentifier(value);
        assertNotNull(id.toUuid());
        id2 = BeaconIdentifier.parse(id.toUuid().toString());
        assertArrayEquals(id.toByteArray(), id2.toByteArray());
        assertEquals(id, id2);
        assertEquals(id.hashCode(), id2.hashCode());

        // force NullPointerException
        id = null;
        try {
            id = new BeaconIdentifier(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(id);
    }

    @Test
    public void testParseFromLong() {
        // create identifier
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long value = randomizer.nextLong();
        final BeaconIdentifier id = BeaconIdentifier.fromLong(value,
                Long.SIZE / Byte.SIZE);

        // force IllegalArgumentException
        try {
            BeaconIdentifier.fromLong(value, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }


        // parse
        final String str = id.toString();
        BeaconIdentifier id2 = BeaconIdentifier.parse(str);
        BeaconIdentifier id3 = BeaconIdentifier.parse(str,
                Long.SIZE / Byte.SIZE);

        assertEquals(id, id2);
        assertEquals(id, id3);

        // force NullPointerException
        try {
            BeaconIdentifier.parse(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // force IllegalArgumentException
        try {
            BeaconIdentifier.parse("hello");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testParseFromInt() {
        // create identifier
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int value = randomizer.nextInt(0, 65535);
        final BeaconIdentifier id = BeaconIdentifier.fromInt(value);

        // force IllegalArgumentException
        try {
            BeaconIdentifier.fromInt(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            BeaconIdentifier.fromInt(65536);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }


        // parse
        final String str = id.toString();
        final BeaconIdentifier id2 = BeaconIdentifier.parse(str);

        assertEquals(id, id2);

        // force NullPointerException
        try {
            BeaconIdentifier.parse(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // force IllegalArgumentException
        try {
            BeaconIdentifier.parse("hello");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testParseFromBytes() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final long value = randomizer.nextLong();
        final BeaconIdentifier id = BeaconIdentifier.fromLong(value,
                Long.SIZE / Byte.SIZE);
        final byte[] bytes = id.toByteArray();

        BeaconIdentifier id2 = BeaconIdentifier.fromBytes(bytes, 0, bytes.length,
                false);
        assertEquals(id, id2);
        assertArrayEquals(id.toByteArray(), id2.toByteArray());

        // test with little endian
        id2 = BeaconIdentifier.fromBytes(bytes, 0, bytes.length, true);
        final byte[] bytes2 = id2.toByteArray();
        for (int i = 0; i < bytes.length; i++) {
            assertEquals(bytes[i], bytes2[bytes.length - 1 - i]);
        }

        // force NullPointerException
        try {
            BeaconIdentifier.fromBytes(null, 0, 1, true);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // force ArrayIndexOutOfBoundsException
        try {
            BeaconIdentifier.fromBytes(bytes, -1, bytes.length, true);
            fail("ArrayIndexOutOfBoundsException expected but not thrown");
        } catch (final ArrayIndexOutOfBoundsException ignore) {
        }
        try {
            BeaconIdentifier.fromBytes(bytes, 0, bytes.length + 1, true);
            fail("ArrayIndexOutOfBoundsException expected but not thrown");
        } catch (final ArrayIndexOutOfBoundsException ignore) {
        }

        // force IllegalArgumentException
        try {
            BeaconIdentifier.fromBytes(bytes, bytes.length, 0, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // force IllegalArgumentException
        try {
            BeaconIdentifier.parse("hello");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromUuid() {
        final byte[] value = new byte[16];

        final Random r = new Random();
        r.nextBytes(value);
        final BeaconIdentifier id = new BeaconIdentifier(value);

        final UUID uuid = id.toUuid();
        BeaconIdentifier id2 = BeaconIdentifier.fromUuid(uuid);

        // check
        assertEquals(id, id2);
    }

    @Test
    public void testToString() {
        final Random r = new Random();
        final byte[] bytes = new byte[16];
        r.nextBytes(bytes);

        final BeaconIdentifier id = BeaconIdentifier.fromBytes(bytes, 0, bytes.length,
                false);

        final String str = id.toString();
        assertEquals("0x" + str.replaceAll("-", ""),
                id.toHexString());

        final BeaconIdentifier id2 = BeaconIdentifier.parse(str);
        assertEquals(id, id2);
    }

    @Test
    public void testParseHexNoPrefix() {
        final Random r = new Random();
        final byte[] bytes = new byte[32];
        r.nextBytes(bytes);

        final BeaconIdentifier id = BeaconIdentifier.fromBytes(bytes, 0, bytes.length,
                false);

        final String str = id.toHexString().substring(2);

        final BeaconIdentifier id2 = BeaconIdentifier.parse(str);
        final BeaconIdentifier id3 = BeaconIdentifier.parse(str, 32);
        assertEquals(id, id2);
        assertEquals(id, id3);

        final BeaconIdentifier id4 = BeaconIdentifier.parse(str, 33);
        assertEquals(str, id4.toHexString().substring(4));

        final BeaconIdentifier id5 = BeaconIdentifier.parse(str, 31);
        assertEquals(str.substring(2), id5.toHexString().substring(2));

    }

    @Test
    public void testParseLongDecimal() {
        final long value = 65536;
        final BeaconIdentifier id = BeaconIdentifier.fromLong(value, 4);

        final BeaconIdentifier id2 = BeaconIdentifier.parse(String.valueOf(value), 4);
        assertEquals(id, id2);
    }

    @Test
    public void testToInt() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int value = randomizer.nextInt(0, 65535);
        BeaconIdentifier id = BeaconIdentifier.fromInt(value);

        assertEquals(id.toInt(), value);

        //force UnsupportedOperationException
        value = 65536;
        id = BeaconIdentifier.fromLong(value, 4);

        try {
            id.toInt();
            fail("UnsupportedOperationException expected but not thrown");
        } catch (final UnsupportedOperationException ignore) {
        }
    }

    @Test
    public void testEqualsAndCompareTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int value = randomizer.nextInt(2, 65535);
        final BeaconIdentifier id = BeaconIdentifier.fromInt(value);
        final BeaconIdentifier id2 = BeaconIdentifier.fromInt(value);
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value, 4);
        final BeaconIdentifier id4 = BeaconIdentifier.fromLong(value, 5);
        final BeaconIdentifier id5 = BeaconIdentifier.fromInt(value - 1);

        assertEquals(id, id2);
        assertEquals(id.compareTo(id2), 0);

        assertNotEquals(id, new Object());

        assertEquals(id3.compareTo(id4), -1);
        assertEquals(id4.compareTo(id3), 1);

        assertEquals(id.compareTo(id5), 1);
        assertEquals(id5.compareTo(id), -1);
    }
}
