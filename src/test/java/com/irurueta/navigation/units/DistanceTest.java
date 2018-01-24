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

public class DistanceTest {

    public DistanceTest() { }

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
        Distance d = new Distance();

        //check
        assertNull(d.getValue());
        assertNull(d.getUnit());

        //test constructor with value and unit
        d = new Distance(323, DistanceUnit.METER);

        //check
        assertEquals(d.getValue(), 323);
        assertEquals(d.getUnit(), DistanceUnit.METER);

        //force IllegalArgumentException
        d = null;
        try {
            d = new Distance(null, DistanceUnit.METER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            d = new Distance(323, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(d);
    }

    @Test
    public void testGetSetValue() {
        Distance d = new Distance(1, DistanceUnit.METER);

        //check
        assertEquals(d.getValue(), 1);

        //set new value
        d.setValue(2.5);

        //check
        assertEquals(d.getValue(), 2.5);

        //force IllegalArgumentException
        try {
            d.setValue(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetUnit() {
        Distance d = new Distance(1, DistanceUnit.METER);

        //check
        assertEquals(d.getUnit(), DistanceUnit.METER);

        //set new value
        d.setUnit(DistanceUnit.INCH);

        //check
        assertEquals(d.getUnit(), DistanceUnit.INCH);

        //force IllegalArgumentException
        try {
            d.setUnit(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
