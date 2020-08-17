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

import static org.junit.Assert.assertEquals;

public class GnomonicDataTest {

    public GnomonicDataTest() {
    }

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    @Test
    public void testConstructor() {
        //empty constructor
        GnomonicData data = new GnomonicData();

        //check getters
        assertEquals(data.getLat0(), Double.NaN, 0.0);
        assertEquals(data.getLon0(), Double.NaN, 0.0);
        assertEquals(data.getLat(), Double.NaN, 0.0);
        assertEquals(data.getLon(), Double.NaN, 0.0);
        assertEquals(data.getX(), Double.NaN, 0.0);
        assertEquals(data.getY(), Double.NaN, 0.0);
        assertEquals(data.getAzi(), Double.NaN, 0.0);
        assertEquals(data.getRk(), Double.NaN, 0.0);


        //constructor with values
        data = new GnomonicData(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);

        //check getters
        assertEquals(data.getLat0(), 0.0, 0.0);
        assertEquals(data.getLon0(), 1.0, 0.0);
        assertEquals(data.getLat(), 2.0, 0.0);
        assertEquals(data.getLon(), 3.0, 0.0);
        assertEquals(data.getX(), 4.0, 0.0);
        assertEquals(data.getY(), 5.0, 0.0);
        assertEquals(data.getAzi(), 6.0, 0.0);
        assertEquals(data.getRk(), 7.0, 0.0);
    }

    @Test
    public void testGetSetLat0() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getLat0(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat0(value);

        //check
        assertEquals(data.getLat0(), value, 0.0);
    }

    @Test
    public void testGetSetLon0() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getLon0(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon0(value);

        //check
        assertEquals(data.getLon0(), value, 0.0);
    }

    @Test
    public void testGetSetLat() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getLat(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat(value);

        //check
        assertEquals(data.getLat(), value, 0.0);
    }

    @Test
    public void testGetSetLon() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getLon(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon(value);

        //check
        assertEquals(data.getLon(), value, 0.0);
    }

    @Test
    public void testGetSetX() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getX(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setX(value);

        //check
        assertEquals(data.getX(), value, 0.0);
    }

    @Test
    public void testGetSetY() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getY(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setY(value);

        //check
        assertEquals(data.getY(), value, 0.0);
    }

    @Test
    public void testGetSetAzi() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getAzi(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAzi(value);

        //check
        assertEquals(data.getAzi(), value, 0.0);
    }

    @Test
    public void testGetSetRk() {
        final GnomonicData data = new GnomonicData();

        //check default value
        assertEquals(data.getRk(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setRk(value);

        //check
        assertEquals(data.getRk(), value, 0.0);
    }
}
