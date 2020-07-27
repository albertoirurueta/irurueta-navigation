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

public class GeodesicDataTest {

    public GeodesicDataTest() {
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
        final GeodesicData data = new GeodesicData();

        //check default values
        assertEquals(data.getLat1(), Double.NaN, 0.0);
        assertEquals(data.getLon1(), Double.NaN, 0.0);
        assertEquals(data.getAzi1(), Double.NaN, 0.0);
        assertEquals(data.getLat2(), Double.NaN, 0.0);
        assertEquals(data.getLon2(), Double.NaN, 0.0);
        assertEquals(data.getAzi2(), Double.NaN, 0.0);
        assertEquals(data.getS12(), Double.NaN, 0.0);
        assertEquals(data.getA12(), Double.NaN, 0.0);
        assertEquals(data.getM12(), Double.NaN, 0.0);
        assertEquals(data.getScaleM12(), Double.NaN, 0.0);
        assertEquals(data.getScaleM21(), Double.NaN, 0.0);
        assertEquals(data.getAreaS12(), Double.NaN, 0.0);
    }

    @Test
    public void testGetsetLat1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getLat1(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat1(value);

        //check
        assertEquals(data.getLat1(), value, 0.0);
    }

    @Test
    public void testGetSetLon1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getLon1(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon1(value);

        //check
        assertEquals(data.getLon1(), value, 0.0);
    }

    @Test
    public void testGetSetAzi1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getAzi1(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAzi1(value);

        //check
        assertEquals(data.getAzi1(), value, 0.0);
    }

    @Test
    public void testGetSetLat2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getLat2(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat2(value);

        //check
        assertEquals(data.getLat2(), value, 0.0);
    }

    @Test
    public void testGetSetLon2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getLon2(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon2(value);

        //check
        assertEquals(data.getLon2(), value, 0.0);
    }

    @Test
    public void testGetSetAzi2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getAzi2(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAzi2(value);

        //check
        assertEquals(data.getAzi2(), value, 0.0);
    }

    @Test
    public void testGetSetS12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getS12(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setS12(value);

        //check
        assertEquals(data.getS12(), value, 0.0);
    }

    @Test
    public void testGetSetA12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getA12(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setA12(value);

        //check
        assertEquals(data.getA12(), value, 0.0);
    }

    @Test
    public void testGetSetM12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getM12(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setM12(value);

        //check
        assertEquals(data.getM12(), value, 0.0);
    }

    @Test
    public void testGetSetScaleM12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getScaleM12(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setScaleM12(value);

        //check
        assertEquals(data.getScaleM12(), value, 0.0);
    }

    @Test
    public void testGetSetScaleM21() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getScaleM21(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setScaleM21(value);

        //check
        assertEquals(data.getScaleM21(), value, 0.0);
    }

    @Test
    public void testGetSetAreaS12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(data.getAreaS12(), Double.NaN, 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAreaS12(value);

        //check
        assertEquals(data.getAreaS12(), value, 0.0);
    }
}
