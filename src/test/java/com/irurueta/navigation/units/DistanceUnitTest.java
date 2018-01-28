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
package com.irurueta.navigation.units;

import org.junit.*;

import static org.junit.Assert.*;

public class DistanceUnitTest {

    public DistanceUnitTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testGetUnitSystem() {
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.MILLIMETER),
                UnitSystem.METRIC);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.CENTIMETER),
                UnitSystem.METRIC);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.METER),
                UnitSystem.METRIC);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.KILOMETER),
                UnitSystem.METRIC);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.INCH),
                UnitSystem.IMPERIAL);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.FOOT),
                UnitSystem.IMPERIAL);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.YARD),
                UnitSystem.IMPERIAL);
        assertEquals(DistanceUnit.getUnitSystem(DistanceUnit.MILE),
                UnitSystem.IMPERIAL);

        //Force IllegalArgumentException
        try {
            DistanceUnit.getUnitSystem(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetMetricUnits() {
        DistanceUnit[] metricUnits = DistanceUnit.getMetricUnits();
        DistanceUnit[] imperialUnits = DistanceUnit.getImperialUnits();

        for (DistanceUnit metricUnit : metricUnits) {
            assertTrue(DistanceUnit.isMetric(metricUnit));
            assertFalse(DistanceUnit.isImperial(metricUnit));
        }

        assertEquals(metricUnits.length + imperialUnits.length, DistanceUnit.values().length);
    }

    @Test
    public void testGetImperialUnits() {
        DistanceUnit[] metricUnits = DistanceUnit.getMetricUnits();
        DistanceUnit[] imperialUnits = DistanceUnit.getImperialUnits();

        for (DistanceUnit imperialUnit : imperialUnits) {
            assertTrue(DistanceUnit.isImperial(imperialUnit));
            assertFalse(DistanceUnit.isMetric(imperialUnit));
        }

        assertEquals(metricUnits.length + imperialUnits.length, DistanceUnit.values().length);
    }

    @Test
    public void testIsMetric() {
        assertTrue(DistanceUnit.isMetric(DistanceUnit.MILLIMETER));
        assertTrue(DistanceUnit.isMetric(DistanceUnit.CENTIMETER));
        assertTrue(DistanceUnit.isMetric(DistanceUnit.METER));
        assertTrue(DistanceUnit.isMetric(DistanceUnit.KILOMETER));
        assertFalse(DistanceUnit.isMetric(DistanceUnit.INCH));
        assertFalse(DistanceUnit.isMetric(DistanceUnit.FOOT));
        assertFalse(DistanceUnit.isMetric(DistanceUnit.YARD));
        assertFalse(DistanceUnit.isMetric(DistanceUnit.MILE));

        //Force IllegalArgumentException
        try {
            //noinspection all
            DistanceUnit.isMetric(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsImperial() {
        assertFalse(DistanceUnit.isImperial(DistanceUnit.MILLIMETER));
        assertFalse(DistanceUnit.isImperial(DistanceUnit.CENTIMETER));
        assertFalse(DistanceUnit.isImperial(DistanceUnit.METER));
        assertFalse(DistanceUnit.isImperial(DistanceUnit.KILOMETER));
        assertTrue(DistanceUnit.isImperial(DistanceUnit.INCH));
        assertTrue(DistanceUnit.isImperial(DistanceUnit.FOOT));
        assertTrue(DistanceUnit.isImperial(DistanceUnit.YARD));
        assertTrue(DistanceUnit.isImperial(DistanceUnit.MILE));

        //Force IllegalArgumentException
        try {
            //noinspection all
            DistanceUnit.isImperial(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
