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

public class GeodesicDataTest {

    @Test
    public void testConstructor() {
        final GeodesicData data = new GeodesicData();

        //check default values
        assertEquals(Double.NaN, data.getLat1(), 0.0);
        assertEquals(Double.NaN, data.getLon1(), 0.0);
        assertEquals(Double.NaN, data.getAzi1(), 0.0);
        assertEquals(Double.NaN, data.getLat2(), 0.0);
        assertEquals(Double.NaN, data.getLon2(), 0.0);
        assertEquals(Double.NaN, data.getAzi2(), 0.0);
        assertEquals(Double.NaN, data.getS12(), 0.0);
        assertEquals(Double.NaN, data.getA12(), 0.0);
        assertEquals(Double.NaN, data.getM12(), 0.0);
        assertEquals(Double.NaN, data.getScaleM12(), 0.0);
        assertEquals(Double.NaN, data.getScaleM21(), 0.0);
        assertEquals(Double.NaN, data.getAreaS12(), 0.0);
    }

    @Test
    public void testGetSetLat1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getLat1(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat1(value);

        //check
        assertEquals(value, data.getLat1(), 0.0);
    }

    @Test
    public void testGetSetLon1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getLon1(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon1(value);

        //check
        assertEquals(value, data.getLon1(), 0.0);
    }

    @Test
    public void testGetSetAzi1() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getAzi1(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAzi1(value);

        //check
        assertEquals(value, data.getAzi1(), 0.0);
    }

    @Test
    public void testGetSetLat2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getLat2(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLat2(value);

        //check
        assertEquals(value, data.getLat2(), 0.0);
    }

    @Test
    public void testGetSetLon2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getLon2(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setLon2(value);

        //check
        assertEquals(value, data.getLon2(), 0.0);
    }

    @Test
    public void testGetSetAzi2() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getAzi2(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAzi2(value);

        //check
        assertEquals(value, data.getAzi2(), 0.0);
    }

    @Test
    public void testGetSetS12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getS12(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setS12(value);

        //check
        assertEquals(value, data.getS12(), 0.0);
    }

    @Test
    public void testGetSetA12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getA12(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setA12(value);

        //check
        assertEquals(value, data.getA12(), 0.0);
    }

    @Test
    public void testGetSetM12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getM12(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setM12(value);

        //check
        assertEquals(value, data.getM12(), 0.0);
    }

    @Test
    public void testGetSetScaleM12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getScaleM12(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setScaleM12(value);

        //check
        assertEquals(value, data.getScaleM12(), 0.0);
    }

    @Test
    public void testGetSetScaleM21() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getScaleM21(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setScaleM21(value);

        //check
        assertEquals(value, data.getScaleM21(), 0.0);
    }

    @Test
    public void testGetSetAreaS12() {
        final GeodesicData data = new GeodesicData();

        //check default value
        assertEquals(Double.NaN, data.getAreaS12(), 0.0);

        //set new value
        final double value = new Random().nextDouble();
        data.setAreaS12(value);

        //check
        assertEquals(value, data.getAreaS12(), 0.0);
    }
}
