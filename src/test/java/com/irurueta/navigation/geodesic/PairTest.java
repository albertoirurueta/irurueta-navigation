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
package com.irurueta.navigation.geodesic;

import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class PairTest {

    public PairTest() { }

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
        Random r = new Random();
        double first = r.nextDouble();
        double second = r.nextDouble();

        Pair p = new Pair(first, second);

        //check
        assertEquals(p.getFirst(), first, 0.0);
        assertEquals(p.getSecond(), second, 0.0);
    }

    @Test
    public void testGetSetFirst() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Pair p = new Pair(value1, 0.0);

        //check
        assertEquals(p.getFirst(), value1, 0.0);

        //set new value
        p.setFirst(value2);

        //check
        assertEquals(p.getFirst(), value2, 0.0);
    }

    @Test
    public void testGetSetSecond() {
        Random r = new Random();
        double value1 = r.nextDouble();
        double value2 = r.nextDouble();

        Pair p = new Pair(0.0, value1);

        //check
        assertEquals(p.getSecond(), value1, 0.0);

        //set new value
        p.setSecond(value2);

        //check
        assertEquals(p.getSecond(), value2, 0.0);
    }
}
