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

public class TimeTest {

    private static final double ERROR = 1e-6;

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
    public void testEquals() {
        double value = new Random().nextDouble();
        Time t1 = new Time(value, TimeUnit.SECOND);
        Time t2 = new Time(value, TimeUnit.SECOND);
        Time t3 = new Time(value + 1.0, TimeUnit.SECOND);
        Time t4 = new Time(value, TimeUnit.MILLISECOND);

        assertEquals(t1, t1);
        assertEquals(t1, t2);
        assertNotEquals(t1, t3);
        assertNotEquals(t1, t4);

        //noinspection all
        assertFalse(t1.equals(null));
        assertFalse(t1.equals(new Object()));
    }

    @Test
    public void testHashCode() {
        double value = new Random().nextDouble();
        Time t1 = new Time(value, TimeUnit.SECOND);
        Time t2 = new Time(value, TimeUnit.SECOND);
        Time t3 = new Time(value + 1.0, TimeUnit.SECOND);
        Time t4 = new Time(value, TimeUnit.MILLISECOND);

        assertEquals(t1.hashCode(), t1.hashCode());
        assertEquals(t1.hashCode(), t2.hashCode());
        assertNotEquals(t1.hashCode(), t3.hashCode());
        assertNotEquals(t1.hashCode(), t4.hashCode());
    }

    @Test
    public void testEqualsWithTolerance() {
        double value = new Random().nextDouble();
        Time t1 = new Time(value, TimeUnit.SECOND);
        Time t2 = new Time(value, TimeUnit.SECOND);
        Time t3 = new Time(value + 0.5 * ERROR, TimeUnit.SECOND);
        Time t4 = new Time(value, TimeUnit.MILLISECOND);
        Time t5 = new Time(value * 1000.0, TimeUnit.MILLISECOND);

        assertTrue(t1.equals(t1, 0.0));
        assertTrue(t1.equals(t2, 0.0));
        assertFalse(t1.equals(t3, 0.0));
        assertTrue(t1.equals(t3, ERROR));
        assertFalse(t1.equals(t4, ERROR));
        assertTrue(t1.equals(t5, ERROR));

        assertFalse(t1.equals(null, ERROR));
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

    @Test
    public void testAdd1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        double result = Time.add(value1, TimeUnit.SECOND,
                value2, TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals((value1 + value2) * 1000.0, result, ERROR);
    }

    @Test
    public void testAdd2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Number result = Time.add(new BigDecimal(value1), TimeUnit.SECOND,
                new BigDecimal(value2), TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals((value1 + value2) * 1000.0, result.doubleValue(), ERROR);
    }

    @Test
    public void testAdd3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = new Time(0.0, TimeUnit.MILLISECOND);
        Time.add(t1, t2, result);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = Time.addAndReturnNew(t1, t2, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        Time result = t1.addAndReturnNew(value2, TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        Time result = t1.addAndReturnNew(new BigDecimal(value2),
                TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAddAndReturnNew4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = t1.addAndReturnNew(t2, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testAdd4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        t1.add(value2, TimeUnit.SECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 + value2,
                ERROR);
    }

    @Test
    public void testAdd5() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        t1.add(new BigDecimal(value2), TimeUnit.SECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 + value2,
                ERROR);
    }


    @Test
    public void testAdd6() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        t1.add(t2);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 + value2,
                ERROR);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);
    }

    @Test
    public void testAdd7() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = new Time(0.0, TimeUnit.MILLISECOND);
        t1.add(t2, result);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 + value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtract1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        double result = Time.subtract(value1, TimeUnit.SECOND,
                value2, TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals((value1 - value2) * 1000.0, result, ERROR);
    }

    @Test
    public void testSubtract2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Number result = Time.subtract(new BigDecimal(value1), TimeUnit.SECOND,
                new BigDecimal(value2), TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals((value1 - value2) * 1000.0, result.doubleValue(), ERROR);
    }

    @Test
    public void testSubtract3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = new Time(0.0, TimeUnit.MILLISECOND);
        Time.subtract(t1, t2, result);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew1() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = Time.subtractAndReturnNew(t1, t2,
                TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew2() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        Time result = t1.subtractAndReturnNew(value2, TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew3() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        Time result = t1.subtractAndReturnNew(new BigDecimal(value2),
                TimeUnit.SECOND, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtractAndReturnNew4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = t1.subtractAndReturnNew(t2, TimeUnit.MILLISECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }

    @Test
    public void testSubtract4() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        t1.subtract(value2, TimeUnit.SECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 - value2,
                ERROR);
    }

    @Test
    public void testSubtract5() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);

        t1.subtract(new BigDecimal(value2), TimeUnit.SECOND);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 - value2,
                ERROR);
    }

    @Test
    public void testSubtract6() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        t1.subtract(t2);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1 - value2,
                ERROR);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);
    }

    @Test
    public void testSubtract7() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Time t1 = new Time(value1, TimeUnit.SECOND);
        Time t2 = new Time(value2, TimeUnit.SECOND);

        Time result = new Time(0.0, TimeUnit.MILLISECOND);
        t1.subtract(t2, result);

        //check
        assertEquals(t1.getUnit(), TimeUnit.SECOND);
        assertEquals(t1.getValue().doubleValue(), value1, 0.0);

        assertEquals(t2.getUnit(), TimeUnit.SECOND);
        assertEquals(t2.getValue().doubleValue(), value2, 0.0);

        assertEquals(result.getUnit(), TimeUnit.MILLISECOND);
        assertEquals((value1 - value2) * 1000.0,
                result.getValue().doubleValue(), ERROR);
    }
}
