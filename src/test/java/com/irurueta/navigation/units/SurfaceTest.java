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

import java.math.BigDecimal;
import java.util.Random;

import static org.junit.Assert.*;

public class SurfaceTest {

    private static final double ERROR = 1e-6;

    public SurfaceTest() { }

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
        Surface s = new Surface();

        //check
        assertNull(s.getValue());
        assertNull(s.getUnit());

        //test constructor with value and unit
        s = new Surface(323, SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s.getValue(), 323);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_METER);

        //force IllegalArgumentException
        s = null;
        try {
            s = new Surface(null, SurfaceUnit.SQUARE_METER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            s = new Surface(323, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(s);
    }

    @Test
    public void testEquals() {
        double value = new Random().nextDouble();
        Surface s1 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s3 = new Surface(value + 1.0, SurfaceUnit.SQUARE_METER);
        Surface s4 = new Surface(value, SurfaceUnit.SQUARE_CENTIMETER);

        assertEquals(s1, s1);
        assertEquals(s1, s2);
        assertNotEquals(s1, s3);
        assertNotEquals(s1, s4);

        //noinspection all
        assertFalse(s1.equals(null));
        assertFalse(s1.equals(new Object()));
    }

    @Test
    public void testHashCode() {
        double value = new Random().nextDouble();
        Surface s1 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s3 = new Surface(value + 1.0, SurfaceUnit.SQUARE_METER);
        Surface s4 = new Surface(value, SurfaceUnit.SQUARE_CENTIMETER);

        assertEquals(s1.hashCode(), s1.hashCode());
        assertEquals(s1.hashCode(), s2.hashCode());
        assertNotEquals(s1.hashCode(), s3.hashCode());
        assertNotEquals(s1.hashCode(), s4.hashCode());
    }

    @Test
    public void testEqualsWithTolerance() {
        double value = new Random().nextDouble();
        Surface s1 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value, SurfaceUnit.SQUARE_METER);
        Surface s3 = new Surface(value + 0.5 * ERROR, SurfaceUnit.SQUARE_METER);
        Surface s4 = new Surface(value, SurfaceUnit.SQUARE_CENTIMETER);
        Surface s5 = new Surface(value * 10000.0, SurfaceUnit.SQUARE_CENTIMETER);

        assertTrue(s1.equals(s1, 0.0));
        assertTrue(s1.equals(s2, 0.0));
        assertFalse(s1.equals(s3, 0.0));
        assertTrue(s1.equals(s3, ERROR));
        assertFalse(s1.equals(s4, ERROR));
        assertTrue(s1.equals(s5, ERROR));

        assertFalse(s1.equals(null, ERROR));
    }

    @Test
    public void testGetSetValue() {
        Surface s = new Surface(1, SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s.getValue(), 1);

        //set new value
        s.setValue(2.5);

        //check
        assertEquals(s.getValue(), 2.5);

        //force IllegalArgumentException
        try {
            s.setValue(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetUnit() {
        Surface s = new Surface(1, SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_METER);

        //set new value
        s.setUnit(SurfaceUnit.SQUARE_YARD);

        //check
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_YARD);

        //force IllegalArgumentException
        try {
            s.setUnit(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testAdd1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        double result = Surface.add(value1, SurfaceUnit.SQUARE_METER,
                value2, SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals((value1 + value2) * 1e-4, result, ERROR);
    }

    @Test
    public void testAdd2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Number result = Surface.add(new BigDecimal(value1), SurfaceUnit.SQUARE_METER,
                new BigDecimal(value2), SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals((value1 + value2) * 1e-4, result.doubleValue(), ERROR);
    }

    @Test
    public void testAdd3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = new Surface(0.0, SurfaceUnit.HECTARE);
        Surface.add(s1, s2, result);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = Surface.addAndReturnNew(s1, s2, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        Surface result = s1.addAndReturnNew(value2, SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        Surface result = s1.addAndReturnNew(new BigDecimal(value2),
                SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = s1.addAndReturnNew(s2, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAdd4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        s1.add(value2, SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 + value2,
                ERROR);
    }

    @Test
    public void testAdd5() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        s1.add(new BigDecimal(value2), SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 + value2,
                ERROR);
    }

    @Test
    public void testAdd6() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        s1.add(s2);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 + value2,
                ERROR);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);
    }

    @Test
    public void testAdd7() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = new Surface(0.0, SurfaceUnit.HECTARE);
        s1.add(s2, result);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 + value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtract1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        double result = Surface.subtract(value1, SurfaceUnit.SQUARE_METER,
                value2, SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals((value1 - value2) * 1e-4, result, ERROR);
    }

    @Test
    public void testSubtract2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Number result = Surface.subtract(new BigDecimal(value1), SurfaceUnit.SQUARE_METER,
                new BigDecimal(value2), SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals((value1 - value2) * 1e-4, result.doubleValue(), ERROR);
    }

    @Test
    public void testSubtract3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = new Surface(0.0, SurfaceUnit.HECTARE);
        Surface.subtract(s1, s2, result);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = Surface.subtractAndReturnNew(s1, s2,
                SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        Surface result = s1.subtractAndReturnNew(value2, SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        Surface result = s1.subtractAndReturnNew(new BigDecimal(value2),
                SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = s1.subtractAndReturnNew(s2, SurfaceUnit.HECTARE);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtract4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        s1.subtract(value2, SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 - value2,
                ERROR);
    }

    @Test
    public void testSubtract5() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);

        s1.subtract(new BigDecimal(value2), SurfaceUnit.SQUARE_METER);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 - value2,
                ERROR);
    }

    @Test
    public void testSubtract6() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        s1.subtract(s2);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1 - value2,
                ERROR);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);
    }

    @Test
    public void testSubtract7() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Surface s1 = new Surface(value1, SurfaceUnit.SQUARE_METER);
        Surface s2 = new Surface(value2, SurfaceUnit.SQUARE_METER);

        Surface result = new Surface(0.0, SurfaceUnit.HECTARE);
        s1.subtract(s2, result);

        //check
        assertEquals(s1.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s1.getValue().doubleValue(), value1, 0.0);

        assertEquals(s2.getUnit(), SurfaceUnit.SQUARE_METER);
        assertEquals(s2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), SurfaceUnit.HECTARE);
        assertEquals((value1 - value2) * 1e-4,
                result.getValue().doubleValue(), ERROR);
    }
}
