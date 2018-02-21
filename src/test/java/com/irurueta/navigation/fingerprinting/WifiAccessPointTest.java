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

public class WifiAccessPointTest {

    private static final double FREQUENCY = 2.4e9;

    public WifiAccessPointTest() { }

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
        WifiAccessPoint ap = new WifiAccessPoint();

        //check default values
        assertNull(ap.getBssid());
        assertNull(ap.getSsid());

        //test constructor with BSSID
        ap = new WifiAccessPoint("bssid", FREQUENCY);

        //check default values
        assertEquals(ap.getBssid(), "bssid");
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());

        //Force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPoint(null, FREQUENCY);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);

        //test constructor with BSSID and SSID
        ap = new WifiAccessPoint("bssid", FREQUENCY,"ssid");

        //check default value
        assertEquals(ap.getBssid(), "bssid");
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), "ssid");

        //Force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPoint(null, FREQUENCY, "ssid");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap2 = new WifiAccessPoint("bssid1", FREQUENCY);
        WifiAccessPoint ap3 = new WifiAccessPoint("bssid2", FREQUENCY);

        //check
        //noinspection all
        assertTrue(ap1.equals(ap1));
        assertTrue(ap1.equals(ap2));
        assertFalse(ap1.equals(ap3));
    }
}
