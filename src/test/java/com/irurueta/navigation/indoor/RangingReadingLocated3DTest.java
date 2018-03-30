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
import org.junit.*;

import static org.junit.Assert.*;

public class RangingReadingLocated3DTest {
    private static final double FREQUENCY = 2.4e9;

    public RangingReadingLocated3DTest() { }

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
        RangingReadingLocated3D<WifiAccessPoint> reading = new RangingReadingLocated3D<>();

        //check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;


        //test constructor with access point, distance and position
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        reading = new RangingReadingLocated3D<>(ap, 1.2, position);

        //check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated3D<>(null,
                    1.2, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 1.2, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, position and distance standard deviation
        reading = new RangingReadingLocated3D<>(ap, 1.5, position,
                0.1);

        //check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated3D<>(null, 1.5, position,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, -1.0, position,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 1.5, null,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 1.5, position,
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, position and position covariance
        Matrix cov = new Matrix(3, 3);
        reading = new RangingReadingLocated3D<>(ap, 2.0, position, cov);

        //check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        reading = new RangingReadingLocated3D<>(ap, 2.0, position,
                (Matrix)null);

        //check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated3D<>(null,
                    2.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, -1.0,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.0,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.0,
                    position, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.0,
                    position, new Matrix(3,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, position, distance standard deviation and position covariance
        reading = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2, cov);

        //check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        reading = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2,
                null);

        //check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);;

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated3D<>(null, 2.5,
                    position, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, -1.0,
                    position, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.5,
                    null, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.5, position,
                    0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.5, position,
                    0.2, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingReadingLocated3D<>(ap, 2.5, position,
                    0.2, new Matrix(3,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);
        InhomogeneousPoint3D position = new InhomogeneousPoint3D();

        RangingReadingLocated3D<WifiAccessPoint> reading1 = new RangingReadingLocated3D<>(ap1,
                50.0, position);
        RangingReadingLocated3D<WifiAccessPoint> reading2 = new RangingReadingLocated3D<>(ap1,
                50.0, position);
        RangingReadingLocated3D<WifiAccessPoint> reading3 = new RangingReadingLocated3D<>(ap2,
                50.0, position);

        //check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }    
}
