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

public class PolygonResultTest {

    public PolygonResultTest() { }

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
        Random random = new Random();
        int num = random.nextInt();
        double perimeter = random.nextDouble();
        double area = random.nextDouble();
        PolygonResult result = new PolygonResult(num, perimeter, area);

        //check
        assertEquals(result.getNum(), num);
        assertEquals(result.getPerimeter(), perimeter, 0.0);
        assertEquals(result.getArea(), area, 0.0);
    }

    @Test
    public void testGetSetNum() {
        Random random = new Random();
        int num1 = random.nextInt();
        int num2 = random.nextInt();
        double perimeter = random.nextDouble();
        double area = random.nextDouble();
        PolygonResult result = new PolygonResult(num1, perimeter, area);

        //check
        assertEquals(result.getNum(), num1);

        //set new value
        result.setNum(num2);

        //check
        assertEquals(result.getNum(), num2);
    }

    @Test
    public void testGetSetPerimeter() {
        Random random = new Random();
        int num = random.nextInt();
        double perimeter1 = random.nextDouble();
        double perimeter2 = random.nextDouble();
        double area = random.nextDouble();
        PolygonResult result = new PolygonResult(num, perimeter1, area);

        //check
        assertEquals(result.getPerimeter(), perimeter1, 0.0);

        //set new value
        result.setPerimeter(perimeter2);

        //check
        assertEquals(result.getPerimeter(), perimeter2, 0.0);
    }

    @Test
    public void testGetSetArea() {
        Random random = new Random();
        int num = random.nextInt();
        double perimeter = random.nextDouble();
        double area1 = random.nextDouble();
        double area2 = random.nextDouble();
        PolygonResult result = new PolygonResult(num, perimeter, area1);

        //check
        assertEquals(result.getArea(), area1, 0.0);

        //set new value
        result.setArea(area2);

        //check
        assertEquals(result.getArea(), area2, 0.0);
    }
}
