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

class PolygonResultTest {

    @Test
    void testConstructor() {
        final var random = new Random();
        final var num = random.nextInt();
        final var perimeter = random.nextDouble();
        final var area = random.nextDouble();
        final var result = new PolygonResult(num, perimeter, area);

        //check
        assertEquals(num, result.getNum());
        assertEquals(perimeter, result.getPerimeter(), 0.0);
        assertEquals(area, result.getArea(), 0.0);
    }

    @Test
    void testGetSetNum() {
        final var random = new Random();
        final var num1 = random.nextInt();
        final var num2 = random.nextInt();
        final var perimeter = random.nextDouble();
        final var area = random.nextDouble();
        final var result = new PolygonResult(num1, perimeter, area);

        //check
        assertEquals(num1, result.getNum());

        //set new value
        result.setNum(num2);

        //check
        assertEquals(num2, result.getNum());
    }

    @Test
    void testGetSetPerimeter() {
        final var random = new Random();
        final var num = random.nextInt();
        final var perimeter1 = random.nextDouble();
        final var perimeter2 = random.nextDouble();
        final var area = random.nextDouble();
        final var result = new PolygonResult(num, perimeter1, area);

        //check
        assertEquals(perimeter1, result.getPerimeter(), 0.0);

        //set new value
        result.setPerimeter(perimeter2);

        //check
        assertEquals(perimeter2, result.getPerimeter(), 0.0);
    }

    @Test
    void testGetSetArea() {
        final var random = new Random();
        final var num = random.nextInt();
        final var perimeter = random.nextDouble();
        final var area1 = random.nextDouble();
        final var area2 = random.nextDouble();
        final var result = new PolygonResult(num, perimeter, area1);

        //check
        assertEquals(area1, result.getArea(), 0.0);

        //set new value
        result.setArea(area2);

        //check
        assertEquals(area2, result.getArea(), 0.0);
    }
}
