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

public class WifiRssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    public WifiRssiReadingTest() { }

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
        WifiRssiReading reading = new WifiRssiReading();

        //check
        assertNull(reading.getAccessPoint());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());


        //test constructor with access point and RSSI
        WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new WifiRssiReading(ap, -50.0);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRssiReading(null, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);


        //test constructor with access point, RSSI and RSSI standard deviation
        reading = new WifiRssiReading(ap, -50.0, 5.5);

        //check
        assertSame(reading.getAccessPoint(), ap);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);

        //Force IllegalArgumentException
        reading = null;
        try {
            reading = new WifiRssiReading(null, -50.0,
                    5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            reading = new WifiRssiReading(ap, -50.0, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        WifiRssiReading reading1 = new WifiRssiReading(ap1, -50.0);
        WifiRssiReading reading2 = new WifiRssiReading(ap1, -50.0);
        WifiRssiReading reading3 = new WifiRssiReading(ap2, -50.0);

        //check
        assertTrue(reading1.hasSameAccessPoint(reading1));
        assertTrue(reading1.hasSameAccessPoint(reading2));
        assertFalse(reading1.hasSameAccessPoint(reading3));
    }
}
