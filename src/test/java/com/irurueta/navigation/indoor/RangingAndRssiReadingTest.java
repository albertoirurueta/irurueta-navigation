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

import org.junit.*;

import static org.junit.Assert.*;

public class RangingAndRssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    public RangingAndRssiReadingTest() { }

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
        RangingAndRssiReading<WifiAccessPoint> reading = new RangingAndRssiReading<>();

        //check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);


        //test constructor with access point, distance and RSSI
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RangingAndRssiReading<>(ap, 1.2, -50.0);

        //check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.2,
                    -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, distance, RSSI and standard deviations
        reading = new RangingAndRssiReading<>(ap, 1.5, -50.0,
                0.1, 5.5);

        //check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.5,
                    -50.0, 0.1, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0,
                    0.1, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.0, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        RangingAndRssiReading<WifiAccessPoint> reading1 = new RangingAndRssiReading<>(ap1,
                1.5, -50.0);
        RangingAndRssiReading<WifiAccessPoint> reading2 = new RangingAndRssiReading<>(ap1,
                1.5, -50.0);
        RangingAndRssiReading<WifiAccessPoint> reading3 = new RangingAndRssiReading<>(ap2,
                1.5, -50.0);

        //check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }
}
