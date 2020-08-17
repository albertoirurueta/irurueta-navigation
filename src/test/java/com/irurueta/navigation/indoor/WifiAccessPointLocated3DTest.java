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
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class WifiAccessPointLocated3DTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    public WifiAccessPointLocated3DTest() {
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
        WifiAccessPointLocated3D ap = new WifiAccessPointLocated3D();

        // check default values
        assertNull(ap.getBssid());
        assertEquals(ap.getFrequency(), 0.0, 0.0);
        assertNull(ap.getSsid());
        assertNull(ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);


        // test constructor with bssid, frequency and position
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, position);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated3D(null, FREQUENCY, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, -FREQUENCY, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);


        // test constructor with bssid, frequency, ssid and position
        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID, position);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated3D(null, FREQUENCY, SSID,
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, -FREQUENCY, SSID,
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID,
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);


        // test constructor with bssid, frequency, position and position covariance
        final Matrix cov = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, position, cov);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, position, null);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated3D(null, FREQUENCY,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, -FREQUENCY, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, null,
                    cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);


        // test constructor with bssid, frequency, ssid, position and
        // position covariance
        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID, position, cov);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID, position,
                null);

        // check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());
        assertEquals(ap.getType(), RadioSourceType.WIFI_ACCESS_POINT);

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated3D(null, FREQUENCY, SSID,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, -FREQUENCY, SSID,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated3D(BSSID, FREQUENCY, SSID,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final WifiAccessPointLocated3D ap1 = new WifiAccessPointLocated3D("bssid1",
                FREQUENCY, position);
        final WifiAccessPointLocated3D ap2 = new WifiAccessPointLocated3D("bssid1",
                FREQUENCY, position);
        final WifiAccessPointLocated3D ap3 = new WifiAccessPointLocated3D("bssid2",
                FREQUENCY, position);

        // check
        assertEquals(ap1, ap1);
        assertEquals(ap1, ap2);
        assertNotEquals(ap1, ap3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final WifiAccessPointLocated3D ap1 = new WifiAccessPointLocated3D("bssid1",
                FREQUENCY, position);
        final WifiAccessPointLocated3D ap2 = new WifiAccessPointLocated3D("bssid1",
                FREQUENCY, position);
        final WifiAccessPointLocated3D ap3 = new WifiAccessPointLocated3D("bssid2",
                FREQUENCY, position);

        // check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }
}
