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

public class TimeTest {

    public TimeTest() { }

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
        Time t = new Time();

        //check
        assertNull(t.getValue());
        assertNull(t.getUnit());

        //test constructor with value and unit
        t = new Time(332, TimeUnit.SECOND);

        //check
        assertEquals(t.getValue(), 332);
        assertEquals(t.getUnit(), TimeUnit.SECOND);

        //force IllegalArgumentException
        t = null;
        try {
            t = new Time(null, TimeUnit.SECOND);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            t = new Time(323, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(t);
    }

    @Test
    public void testGetSetValue() {
        Time t = new Time(1, TimeUnit.SECOND);

        //check
        assertEquals(t.getValue(), 1);

        //set new value
        t.setValue(2.5);

        //check
        assertEquals(t.getValue(), 2.5);

        //force IllegalArgumentException
        try {
            t.setValue(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetUnit() {
        Time t = new Time(1, TimeUnit.SECOND);

        //check
        assertEquals(t.getUnit(), TimeUnit.SECOND);

        //set new value
        t.setUnit(TimeUnit.DAY);

        //check
        assertEquals(t.getUnit(), TimeUnit.DAY);

        //force IllegalArgumentException
        try {
            t.setUnit(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
