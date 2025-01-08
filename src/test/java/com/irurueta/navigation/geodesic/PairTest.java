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

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

class PairTest {

    @Test
    void testConstructor() {
        final var r = new Random();
        final var first = r.nextDouble();
        final var second = r.nextDouble();

        final var p = new Pair(first, second);

        //check
        assertEquals(first, p.getFirst(), 0.0);
        assertEquals(second, p.getSecond(), 0.0);
    }

    @Test
    void testGetSetFirst() {
        final var r = new Random();
        final var value1 = r.nextDouble();
        final var value2 = r.nextDouble();

        final var p = new Pair(value1, 0.0);

        //check
        assertEquals(value1, p.getFirst(), 0.0);

        //set new value
        p.setFirst(value2);

        //check
        assertEquals(value2, p.getFirst(), 0.0);
    }

    @Test
    void testGetSetSecond() {
        final var r = new Random();
        final var value1 = r.nextDouble();
        final var value2 = r.nextDouble();

        final var p = new Pair(0.0, value1);

        //check
        assertEquals(value1, p.getSecond(), 0.0);

        //set new value
        p.setSecond(value2);

        //check
        assertEquals(value2, p.getSecond(), 0.0);
    }
}
