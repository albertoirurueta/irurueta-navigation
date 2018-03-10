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
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint3D;
import org.junit.*;

import static org.junit.Assert.*;

public class WifiRangingAndRssiReadingLocated3DTest {
    private static final double FREQUENCY = 2.4e9;

    public WifiRangingAndRssiReadingLocated3DTest() { }

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
        WifiRangingAndRssiReadingLocated3D reading = new WifiRangingAndRssiReadingLocated3D();

        //check
        assertNull(reading.getAccessPoint());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());


        //test constructor with access point, distance, rssi and position
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.2,
                -50.0, position);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(null,
                    1.2, -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, -1.0,
                    -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.2,
                    -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, rssi, position,
        //distance standard deviation and rssi standard deviation
        reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.5, -50.0,
                position, 0.1, 0.2);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());

        reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.5, -50.0,
                position, null, null);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(null, 1.5, -50.0,
                    position, 0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, -1.0, -50.0,
                    position, 0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.5, -50.0,
                    null, 0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.5, -50.0,
                    position, 0.0, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 1.5, -50.0,
                    position, 0.1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, rssi, position and position covariance
        Matrix cov = new Matrix(3, 3);
        reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.0, -50.0,
                position, cov);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(null,
                    2.0, -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, -1.0, -50.0,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.0, -50.0,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.0, -50.0,
                    position, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, rssi, position,
        //distance standard deviation, rssi standard deviation and position
        //covariance
        reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                position, 0.1, 0.2, cov);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);

        reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                position, null, null, null);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(null, 2.5, -50.0,
                    position, 0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, -1.0, -50.0,
                    position, 0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                    null, 0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                    position, 0.0, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                    position, 0.1, 0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingAndRssiReadingLocated3D(ap, 2.5, -50.0,
                    position, 0.1, 0.2,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        InhomogeneousPoint3D position = new InhomogeneousPoint3D();

        WifiRangingAndRssiReadingLocated3D reading1 = new WifiRangingAndRssiReadingLocated3D(
                ap1, 1.5, -50.0, position);
        WifiRangingAndRssiReadingLocated3D reading2 = new WifiRangingAndRssiReadingLocated3D(
                ap1, 1.5, -50.0, position);
        WifiRangingAndRssiReadingLocated3D reading3 = new WifiRangingAndRssiReadingLocated3D(
                ap2, 1.5, -50.0, position);

        //check
        assertTrue(reading1.hasSameAccessPoint(reading1));
        assertTrue(reading1.hasSameAccessPoint(reading2));
        assertFalse(reading1.hasSameAccessPoint(reading3));
    }
}
