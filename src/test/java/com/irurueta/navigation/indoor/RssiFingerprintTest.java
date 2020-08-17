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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RssiFingerprintTest {

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_READINGS = 1;
    private static final int MAX_READINGS = 5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double FREQUENCY = 2.4e9;

    public RssiFingerprintTest() {
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
    public void testConstructor() {
        // test empty constructor
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f =
                new RssiFingerprint<>();

        // check default values
        assertTrue(f.getReadings().isEmpty());


        // test constructor with readings
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        f = new RssiFingerprint<>(readings);

        // check
        assertSame(f.getReadings(), readings);

        // force IllegalArgumentException
        f = null;
        try {
            f = new RssiFingerprint<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(f);
    }

    @Test
    public void testGetSetReadings() {
        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f = new RssiFingerprint<>();

        // check default value
        assertTrue(f.getReadings().isEmpty());

        // set new value
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        f.setReadings(readings);

        // check
        assertSame(f.getReadings(), readings);

        // force IllegalArgumentException
        try {
            f.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testDistanceToAndSqrDistanceTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test fingerprint with empty readings
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f = new RssiFingerprint<>();

        assertEquals(f.sqrDistanceTo(f), Double.MAX_VALUE, 0.0);

        // test equal fingerprints
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final int rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        f = new RssiFingerprint<>(readings);

        assertEquals(f.sqrDistanceTo(f), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f), 0.0, ABSOLUTE_ERROR);

        // test different fingerprint RSSI values
        final List<RssiReading<WifiAccessPoint>> readings2 = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final double rssi = readings.get(i).getRssi() + 1.0;
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings2.add(reading);
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f2 =
                new RssiFingerprint<>(readings2);

        assertEquals(f.sqrDistanceTo(f2), numReadings, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f2), Math.sqrt(numReadings), ABSOLUTE_ERROR);

        // test different fingerprint access points
        final List<RssiReading<WifiAccessPoint>> readings3 = new ArrayList<>();
        for (int i = 0; i < numReadings + 1; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi;
            if (i < numReadings) {
                rssi = readings.get(i).getRssi();
            } else {
                rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            }
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings3.add(reading);
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f3 =
                new RssiFingerprint<>(readings3);

        assertEquals(f.sqrDistanceTo(f3), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.distanceTo(f3), 0.0, ABSOLUTE_ERROR);

        // test with null fingerprints
        assertEquals(f.sqrDistanceTo(null), Double.MAX_VALUE,
                0.0);
    }

    @Test
    public void testGetMeanRssi() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        double meanRssi = 0.0;
        for (int i = 0; i < numReadings; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final int rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            meanRssi += (double) rssi / (double) numReadings;

            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f =
                new RssiFingerprint<>(readings);
        assertEquals(f.getMeanRssi(), meanRssi, ABSOLUTE_ERROR);
    }

    @Test
    public void testNoMeanDistanceToAndSqrDistanceTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test fingerprint with empty readings
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f = new RssiFingerprint<>();

        assertEquals(f.noMeanSqrDistanceTo(f), Double.MAX_VALUE, 0.0);

        // test equal fingerprints
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final int rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        f = new RssiFingerprint<>(readings);

        assertEquals(f.noMeanSqrDistanceTo(f), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.noMeanDistanceTo(f), 0.0, ABSOLUTE_ERROR);

        // test different fingerprint RSSI values
        final List<RssiReading<WifiAccessPoint>> readings2 = new ArrayList<>();
        for (int i = 0; i < numReadings; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final double rssi = readings.get(i).getRssi() + 1.0;
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings2.add(reading);
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f2 =
                new RssiFingerprint<>(readings2);

        assertEquals(f.noMeanSqrDistanceTo(f2), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.noMeanDistanceTo(f2), 0.0, ABSOLUTE_ERROR);

        // test different fingerprint access points
        final List<RssiReading<WifiAccessPoint>> readings3 = new ArrayList<>();
        for (int i = 0; i < numReadings + 1; i++) {
            final WifiAccessPoint ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi;
            if (i < numReadings) {
                rssi = readings.get(i).getRssi();
            } else {
                rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            }
            final RssiReading<WifiAccessPoint> reading = new RssiReading<>(ap, rssi);
            readings3.add(reading);
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f3 =
                new RssiFingerprint<>(readings3);

        assertEquals(f.noMeanSqrDistanceTo(f3), 0.0, ABSOLUTE_ERROR);
        assertEquals(f.noMeanDistanceTo(f3), 0.0, ABSOLUTE_ERROR);

        // test with null fingerprints
        assertEquals(f.noMeanSqrDistanceTo(null), Double.MAX_VALUE,
                0.0);
    }
}
