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

import org.junit.*;

import static org.junit.Assert.*;

public class WifiRangingReadingTest {

    private static final double FREQUENCY = 2.4e9;

    public WifiRangingReadingTest() { }

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
        WifiRangingReading<WifiAccessPoint> reading = new WifiRangingReading<>();

        //check
        assertNull(reading.getAccessPoint());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());


        //test constructor with access point and distance
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new WifiRangingReading<>(ap, 1.2);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingReading<>(null, 1.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingReading<>(ap, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance and distance standard deviation
        reading = new WifiRangingReading<>(ap, 1.5, 0.1);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);

        reading = new WifiRangingReading<>(ap, 1.5, null);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());

        //force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRangingReading<>(null, 1.5,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingReading<>(ap, -1.0,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRangingReading<>(ap, 1.5,
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        WifiRangingReading<WifiAccessPoint> reading1 = new WifiRangingReading<>(ap1, 50.0);
        WifiRangingReading<WifiAccessPoint> reading2 = new WifiRangingReading<>(ap1, 50.0);
        WifiRangingReading<WifiAccessPoint> reading3 = new WifiRangingReading<>(ap2, 50.0);

        //check
        assertTrue(reading1.hasSameAccessPoint(reading1));
        assertTrue(reading1.hasSameAccessPoint(reading2));
        assertFalse(reading1.hasSameAccessPoint(reading3));
    }
}
