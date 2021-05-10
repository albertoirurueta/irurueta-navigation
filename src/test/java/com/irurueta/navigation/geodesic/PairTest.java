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

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class PairTest {

    @Test
    public void testConstructor() {
        final Random r = new Random();
        final double first = r.nextDouble();
        final double second = r.nextDouble();

        final Pair p = new Pair(first, second);

        //check
        assertEquals(p.getFirst(), first, 0.0);
        assertEquals(p.getSecond(), second, 0.0);
    }

    @Test
    public void testGetSetFirst() {
        final Random r = new Random();
        final double value1 = r.nextDouble();
        final double value2 = r.nextDouble();

        final Pair p = new Pair(value1, 0.0);

        //check
        assertEquals(p.getFirst(), value1, 0.0);

        //set new value
        p.setFirst(value2);

        //check
        assertEquals(p.getFirst(), value2, 0.0);
    }

    @Test
    public void testGetSetSecond() {
        final Random r = new Random();
        final double value1 = r.nextDouble();
        final double value2 = r.nextDouble();

        final Pair p = new Pair(0.0, value1);

        //check
        assertEquals(p.getSecond(), value1, 0.0);

        //set new value
        p.setSecond(value2);

        //check
        assertEquals(p.getSecond(), value2, 0.0);
    }
}
