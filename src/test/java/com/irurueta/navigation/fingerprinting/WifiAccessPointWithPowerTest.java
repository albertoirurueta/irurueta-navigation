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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class WifiAccessPointWithPowerTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double TRANSMITTED_POWER = -50.0;
    private static final double TRANSMITTED_POWER_STD = 0.5;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    public WifiAccessPointWithPowerTest() { }

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
        WifiAccessPointWithPower ap = new WifiAccessPointWithPower();

        //check default values
        assertNull(ap.getBssid());
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), 0.0, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertEquals(ap.getPathLossExponent(),
                WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);


        //test with bssid, frequency and transmitted power
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY,
                TRANSMITTED_POWER);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertEquals(ap.getPathLossExponent(),
                WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY,
                    TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid and transmitted power
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertEquals(ap.getPathLossExponent(),
                WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID,
                    TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID,
                    TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, transmitted power, transmitted power
        // standard deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertEquals(ap.getPathLossExponent(),
                WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid, transmitted power and transmitted power
        //standard deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertEquals(ap.getPathLossExponent(),
                WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);

        //test with bssid, frequency, transmitted power, transmitted power standard
        //deviation and path loss exponent
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double pathLossExponent = randomizer.nextDouble(
                MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertEquals(ap.getPathLossExponent(), pathLossExponent, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, transmitted power, transmitted power standard
        //deviation and path loss exponent
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertEquals(ap.getPathLossExponent(), pathLossExponent, 0.0);

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        WifiAccessPointWithPower ap1 = new WifiAccessPointWithPower("bssid1",
                FREQUENCY, TRANSMITTED_POWER);
        WifiAccessPointWithPower ap2 = new WifiAccessPointWithPower("bssid1",
                FREQUENCY, TRANSMITTED_POWER);
        WifiAccessPointWithPower ap3 = new WifiAccessPointWithPower("bssid2",
                FREQUENCY, TRANSMITTED_POWER);

        //check
        //noinspection all
        assertTrue(ap1.equals(ap1));
        assertTrue(ap1.equals(ap2));
        assertFalse(ap1.equals(ap3));
    }

    @Test
    public void testHashCode() {
        WifiAccessPointWithPower ap1 = new WifiAccessPointWithPower("bssid1",
                FREQUENCY, TRANSMITTED_POWER);
        WifiAccessPointWithPower ap2 = new WifiAccessPointWithPower("bssid1",
                FREQUENCY, TRANSMITTED_POWER);
        WifiAccessPointWithPower ap3 = new WifiAccessPointWithPower("bssid2",
                FREQUENCY, TRANSMITTED_POWER);

        //check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }
}
