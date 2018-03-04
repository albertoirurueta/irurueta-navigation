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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WifiFingerprintTest {

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_READINGS = 1;
    private static final int MAX_READINGS = 5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double FREQUENCY = 2.4e9;

    public WifiFingerprintTest() { }

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
        WifiFingerprint f = new WifiFingerprint();

        //check default values
        assertTrue(f.getReadings().isEmpty());


        //test constructor with readings
        List<WifiReading> readings = new ArrayList<>();
        f = new WifiFingerprint(readings);

        //check
        assertSame(f.getReadings(), readings);

        //force IllegalArgumentException
        f = null;
        try {
            f = new WifiFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(f);
    }

    @Test
    public void testGetSetReadings() {
        WifiFingerprint f = new WifiFingerprint();

        //check default value
        assertTrue(f.getReadings().isEmpty());

        //set new value
        List<WifiReading> readings = new ArrayList<>();
        f.setReadings(readings);

        //check
        assertSame(f.getReadings(), readings);

        //force IllegalArgumentException
        try {
            f.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testDistanceToAndSqrDistanceTo() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //test fingerprint with empty readings
        WifiFingerprint f = new WifiFingerprint();

        assertEquals(f.sqrDistanceTo(f), Double.MAX_VALUE, 0.0);

        //test equal fingerprints
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        List<WifiReading> readings = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            int rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            WifiReading reading = new WifiReading(ap, rssi);
            readings.add(reading);
        }

        f = new WifiFingerprint(readings);

        assertEquals(f.sqrDistanceTo(f), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f), 0.0, ABSOLUTE_ERROR);

        //test different fingerprint RSSI values
        List<WifiReading> readings2 = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi = readings.get(i).getRssi() + 1.0;
            WifiReading reading = new WifiReading(ap, rssi);
            readings2.add(reading);
        }

        WifiFingerprint f2 = new WifiFingerprint(readings2);

        assertEquals(f.sqrDistanceTo(f2), numReadings, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f2), Math.sqrt(numReadings), ABSOLUTE_ERROR);

        //test different fingerprint access points
        List<WifiReading> readings3 = new ArrayList<>();
        for (int i = 0; i < numReadings + 1; i++) {
            WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi;
            if (i < numReadings) {
                rssi = readings.get(i).getRssi();
            } else {
                rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            }
            WifiReading reading = new WifiReading(ap, rssi);
            readings3.add(reading);
        }

        WifiFingerprint f3 = new WifiFingerprint(readings3);

        assertEquals(f.sqrDistanceTo(f3), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f3), 0.0, ABSOLUTE_ERROR);

        //test with null fingerprints
        assertEquals(f.sqrDistanceTo(null), Double.MAX_VALUE,
                0.0);
    }
}