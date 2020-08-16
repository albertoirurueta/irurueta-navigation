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
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.*;

public class RssiReadingLocated3DTest {

    private static final double FREQUENCY = 2.4e9;

    public RssiReadingLocated3DTest() {
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
        RssiReadingLocated3D<WifiAccessPoint> reading = new RssiReadingLocated3D<>();

        // check
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertNull(reading.getSource());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);


        // test constructor with access point, rssi and position
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        reading = new RssiReadingLocated3D<>(ap, -50.0, position);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReadingLocated3D<>(null, -50.0,
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);


        // test constructor with access point, rssi, position and rssi standard
        // deviation
        reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                5.5);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReadingLocated3D<>(null, -50.0, position,
                    5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, null,
                    5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);


        // test constructor with access point, rssi, position and covariance
        final Matrix cov = new Matrix(3, 3);
        reading = new RssiReadingLocated3D<>(ap, -50.0, position, cov);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        reading = new RssiReadingLocated3D<>(ap, -50.0, position, (Matrix) null);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReadingLocated3D<>(null, -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);


        // test constructor with access point, rssi, position, rssi standard
        // deviation, and position covariance
        reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                5.5, cov);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReadingLocated3D<>(null, -50.0,
                    position, 5.5, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, null,
                    5.5, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                    0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RssiReadingLocated3D<>(ap, -50.0, position,
                    5.5, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        final WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        final RssiReadingLocated3D<WifiAccessPoint> reading1 = new RssiReadingLocated3D<>(ap1,
                -50.0, position);
        RssiReadingLocated3D<WifiAccessPoint> reading2 = new RssiReadingLocated3D<>(ap1,
                -50.0, position);
        RssiReadingLocated3D<WifiAccessPoint> reading3 = new RssiReadingLocated3D<>(ap2,
                -50.0, position);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

}
