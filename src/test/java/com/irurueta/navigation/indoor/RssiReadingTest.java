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

public class RssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    public RssiReadingTest() { }

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
        RssiReading<WifiAccessPoint> reading = new RssiReading<>();

        //check
        assertNull(reading.getSource());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);


        //test constructor with access point and RSSI
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RssiReading<>(ap, -50.0);

        //check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReading<>(null, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, RSSI and RSSI standard deviation
        reading = new RssiReading<>(ap, -50.0, 5.5);

        //check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RSSI_READING);

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new RssiReading<>(null, -50.0,
                    5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new RssiReading<>(ap, -50.0, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        RssiReading<WifiAccessPoint> reading1 = new RssiReading<>(ap1, -50.0);
        RssiReading<WifiAccessPoint> reading2 = new RssiReading<>(ap1, -50.0);
        RssiReading<WifiAccessPoint> reading3 = new RssiReading<>(ap2, -50.0);

        //check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }
}
