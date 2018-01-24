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

public class SurfaceUnitTest {

    public SurfaceUnitTest() { }

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
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_MILLIMETER),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_CENTIMETER),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_METER),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_KILOMETER),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_INCH),
                UnitSystem.IMPERIAL);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_FOOT),
                UnitSystem.IMPERIAL);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_YARD),
                UnitSystem.IMPERIAL);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.SQUARE_MILE),
                UnitSystem.IMPERIAL);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.CENTIARE),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.ARE),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.DECARE),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.HECTARE),
                UnitSystem.METRIC);
        assertEquals(SurfaceUnit.getUnitSystem(SurfaceUnit.ACRE),
                UnitSystem.IMPERIAL);

        //force IllegalArgumentException
        try {
            SurfaceUnit.getUnitSystem(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetMetricUnits() {
        SurfaceUnit[] metricUnits = SurfaceUnit.getMetricUnits();
        SurfaceUnit[] imperialUnits = SurfaceUnit.getImperialUnits();

        for (SurfaceUnit metricUnit : metricUnits) {
            assertTrue(SurfaceUnit.isMetric(metricUnit));
            assertFalse(SurfaceUnit.isImperial(metricUnit));
        }

        assertEquals(metricUnits.length + imperialUnits.length, SurfaceUnit.values().length);
    }

    @Test
    public void testGetImperialUnits() {
        SurfaceUnit[] metricUnits = SurfaceUnit.getMetricUnits();
        SurfaceUnit[] imperialUnits = SurfaceUnit.getImperialUnits();

        for (SurfaceUnit imperialUnit : imperialUnits) {
            assertTrue(SurfaceUnit.isImperial(imperialUnit));
            assertFalse(SurfaceUnit.isMetric(imperialUnit));
        }

        assertEquals(metricUnits.length + imperialUnits.length, SurfaceUnit.values().length);
    }

    @Test
    public void testIsMetric() {
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_MILLIMETER));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_CENTIMETER));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_METER));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_KILOMETER));
        assertFalse(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_INCH));
        assertFalse(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_FOOT));
        assertFalse(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_YARD));
        assertFalse(SurfaceUnit.isMetric(SurfaceUnit.SQUARE_MILE));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.CENTIARE));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.ARE));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.DECARE));
        assertTrue(SurfaceUnit.isMetric(SurfaceUnit.HECTARE));
        assertFalse(SurfaceUnit.isMetric(SurfaceUnit.ACRE));

        //force IllegalArgumentException
        try {
            SurfaceUnit.isMetric(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsImperial() {
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_MILLIMETER));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_CENTIMETER));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_METER));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_KILOMETER));
        assertTrue(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_INCH));
        assertTrue(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_FOOT));
        assertTrue(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_YARD));
        assertTrue(SurfaceUnit.isImperial(SurfaceUnit.SQUARE_MILE));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.CENTIARE));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.ARE));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.DECARE));
        assertFalse(SurfaceUnit.isImperial(SurfaceUnit.HECTARE));
        assertTrue(SurfaceUnit.isImperial(SurfaceUnit.ACRE));

        //force IllegalArgumentException
        try {
            SurfaceUnit.isImperial(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
