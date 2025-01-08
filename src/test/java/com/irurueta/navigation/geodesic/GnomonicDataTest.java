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

class GnomonicDataTest {

    @Test
    void testConstructor() {
        //empty constructor
        var data = new GnomonicData();

        //check getters
        assertEquals(Double.NaN, data.getLat0(), 0.0);
        assertEquals(Double.NaN, data.getLon0(), 0.0);
        assertEquals(Double.NaN, data.getLat(), 0.0);
        assertEquals(Double.NaN, data.getLon(), 0.0);
        assertEquals(Double.NaN, data.getX(), 0.0);
        assertEquals(Double.NaN, data.getY(), 0.0);
        assertEquals(Double.NaN, data.getAzi(), 0.0);
        assertEquals(Double.NaN, data.getRk(), 0.0);

        //constructor with values
        data = new GnomonicData(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);

        //check getters
        assertEquals(0.0, data.getLat0(), 0.0);
        assertEquals(1.0, data.getLon0(), 0.0);
        assertEquals(2.0, data.getLat(), 0.0);
        assertEquals(3.0, data.getLon(), 0.0);
        assertEquals(4.0, data.getX(), 0.0);
        assertEquals(5.0, data.getY(), 0.0);
        assertEquals(6.0, data.getAzi(), 0.0);
        assertEquals(7.0, data.getRk(), 0.0);
    }

    @Test
    void testGetSetLat0() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getLat0(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setLat0(value);

        //check
        assertEquals(value, data.getLat0(), 0.0);
    }

    @Test
    void testGetSetLon0() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getLon0(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setLon0(value);

        //check
        assertEquals(value, data.getLon0(), 0.0);
    }

    @Test
    void testGetSetLat() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getLat(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setLat(value);

        //check
        assertEquals(value, data.getLat(), 0.0);
    }

    @Test
    void testGetSetLon() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getLon(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setLon(value);

        //check
        assertEquals(value, data.getLon(), 0.0);
    }

    @Test
    void testGetSetX() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getX(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setX(value);

        //check
        assertEquals(value, data.getX(), 0.0);
    }

    @Test
    void testGetSetY() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getY(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setY(value);

        //check
        assertEquals(value, data.getY(), 0.0);
    }

    @Test
    void testGetSetAzi() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getAzi(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setAzi(value);

        //check
        assertEquals(value, data.getAzi(), 0.0);
    }

    @Test
    void testGetSetRk() {
        final var data = new GnomonicData();

        //check default value
        assertEquals(Double.NaN, data.getRk(), 0.0);

        //set new value
        final var value = new Random().nextDouble();
        data.setRk(value);

        //check
        assertEquals(value, data.getRk(), 0.0);
    }
}
