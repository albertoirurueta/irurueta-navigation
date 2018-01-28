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

public class TimeUnitTest {

    public TimeUnitTest() { }

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
        assertEquals(TimeUnit.getUnitSystem(TimeUnit.NANOSECOND),
                UnitSystem.METRIC);
        assertEquals(TimeUnit.getUnitSystem(TimeUnit.MICROSECOND),
                UnitSystem.METRIC);
        assertEquals(TimeUnit.getUnitSystem(TimeUnit.MILLISECOND),
                UnitSystem.METRIC);
        assertEquals(TimeUnit.getUnitSystem(TimeUnit.SECOND),
                UnitSystem.METRIC);

        //force IllegalArgumentException
        try {
            TimeUnit.getUnitSystem(TimeUnit.MINUTE);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.HOUR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.DAY);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.WEEK);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.MONTH);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.YEAR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(TimeUnit.CENTURY);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            TimeUnit.getUnitSystem(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetMetricUnits() {
        TimeUnit[] metricUnits = TimeUnit.getMetricUnits();
        TimeUnit[] nonISUnits = TimeUnit.getNonInternationalSystemUnits();

        for (TimeUnit metricUnit : metricUnits) {
            assertTrue(TimeUnit.isMetric(metricUnit));
            assertFalse(TimeUnit.isNonInternationalSystem(metricUnit));
        }

        assertEquals(metricUnits.length + nonISUnits.length, TimeUnit.values().length);
    }

    @Test
    public void testGetNonInternationalSystemUnits() {
        TimeUnit[] metricUnits = TimeUnit.getMetricUnits();
        TimeUnit[] nonISUnits = TimeUnit.getNonInternationalSystemUnits();

        for (TimeUnit nonSIUnit : nonISUnits) {
            assertTrue(TimeUnit.isNonInternationalSystem(nonSIUnit));
            assertFalse(TimeUnit.isMetric(nonSIUnit));
        }

        assertEquals(metricUnits.length + nonISUnits.length, TimeUnit.values().length);
    }

    @Test
    public void testIsMetric() {
        assertTrue(TimeUnit.isMetric(TimeUnit.NANOSECOND));
        assertTrue(TimeUnit.isMetric(TimeUnit.MICROSECOND));
        assertTrue(TimeUnit.isMetric(TimeUnit.MILLISECOND));
        assertTrue(TimeUnit.isMetric(TimeUnit.SECOND));
        assertFalse(TimeUnit.isMetric(TimeUnit.MINUTE));
        assertFalse(TimeUnit.isMetric(TimeUnit.HOUR));
        assertFalse(TimeUnit.isMetric(TimeUnit.DAY));
        assertFalse(TimeUnit.isMetric(TimeUnit.WEEK));
        assertFalse(TimeUnit.isMetric(TimeUnit.MONTH));
        assertFalse(TimeUnit.isMetric(TimeUnit.YEAR));
        assertFalse(TimeUnit.isMetric(TimeUnit.CENTURY));

        //Force IllegalArgumentException
        try {
            TimeUnit.isMetric(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsNonInternationalSystem() {
        assertFalse(TimeUnit.isNonInternationalSystem(TimeUnit.NANOSECOND));
        assertFalse(TimeUnit.isNonInternationalSystem(TimeUnit.MICROSECOND));
        assertFalse(TimeUnit.isNonInternationalSystem(TimeUnit.MILLISECOND));
        assertFalse(TimeUnit.isNonInternationalSystem(TimeUnit.SECOND));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.MINUTE));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.HOUR));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.DAY));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.WEEK));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.MONTH));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.YEAR));
        assertTrue(TimeUnit.isNonInternationalSystem(TimeUnit.CENTURY));

        //Force IllegalArgumentException
        try {
            TimeUnit.isNonInternationalSystem(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
