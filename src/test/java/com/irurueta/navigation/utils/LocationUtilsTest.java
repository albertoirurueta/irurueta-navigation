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
package com.irurueta.navigation.utils;

import com.irurueta.navigation.geodesic.Geodesic;
import com.irurueta.navigation.geodesic.GeodesicData;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Locale;

import static org.junit.Assert.*;

public class LocationUtilsTest {

    @BeforeClass
    public static void setUpClass() {
        Locale.setDefault(Locale.ENGLISH);
    }

    @Test
    public void testConvertDoubleToString() {
        String str = LocationUtils.convert(45.0, LocationUtils.FORMAT_DEGREES);
        assertEquals("45", str);


        str = LocationUtils.convert(45.5, LocationUtils.FORMAT_DEGREES);
        assertEquals("45.5", str);


        str = LocationUtils.convert(45.0, LocationUtils.FORMAT_MINUTES);
        assertEquals("45:0", str);


        str = LocationUtils.convert(45.5, LocationUtils.FORMAT_MINUTES);
        assertEquals("45:30", str);


        str = LocationUtils.convert(45.54, LocationUtils.FORMAT_SECONDS);
        assertEquals("45:32:24", str);

        str = LocationUtils.convert(-45.0, LocationUtils.FORMAT_DEGREES);
        assertEquals("-45", str);

        // force IllegalArgumentException
        str = null;
        try {
            str = LocationUtils.convert(-181.0, LocationUtils.FORMAT_DEGREES);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            str = LocationUtils.convert(181.0, LocationUtils.FORMAT_DEGREES);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            str = LocationUtils.convert(45.0, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(str);
    }

    @Test
    public void testConvertStringToDouble() {
        double value = LocationUtils.convert("45");
        assertEquals(45.0, value, 0.0);

        value = LocationUtils.convert("45.5");
        assertEquals(45.5, value, 0.0);

        value = LocationUtils.convert("45:0");
        assertEquals(45.0, value, 0.0);

        value = LocationUtils.convert("45:30");
        assertEquals(45.5, value, 0.0);

        value = LocationUtils.convert("45:32:24");
        assertEquals(45.54, value, 0.0);

        value = LocationUtils.convert("-45");
        assertEquals(-45.0, value, 0.0);

        // force NullPointerException
        value = 0.0;
        try {
            value = LocationUtils.convert(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // force IllegalArgumentException
        try {
            value = LocationUtils.convert(" ");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("m");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("--1:30");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertEquals(0.0, value, 0.0);
        try {
            value = LocationUtils.convert("181:30");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("45:-1");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("45:60");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("45:30:-1");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = LocationUtils.convert("45:30:60");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertEquals(0.0, value, 0.0);
    }

    @Test
    public void testDistanceAndBearing() {
        // define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        // 41.382643,2.176700
        // 41.382524,2.176861

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        final GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        final LocationUtils.BearingDistance bd1 = new LocationUtils.BearingDistance();
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, bd1);

        final LocationUtils.BearingDistance bd2 = LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2);

        // check
        assertEquals(lat1, bd1.getStartLatitude(), 0.0);
        assertEquals(lon1, bd1.getStartLongitude(), 0.0);
        assertEquals(lat2, bd1.getEndLatitude(), 0.0);
        assertEquals(lon2, bd1.getEndLongitude(), 0.0);
        assertEquals(data.getS12(), bd1.getDistanceMeters(), 0.0);
        assertEquals(data.getAzi1(), bd1.getInitialBearing(), 0.0);
        assertEquals(data.getAzi2(), bd1.getFinalBearing(), 0.0);

        assertEquals(lat1, bd2.getStartLatitude(), 0.0);
        assertEquals(lon1, bd2.getStartLongitude(), 0.0);
        assertEquals(lat2, bd2.getEndLatitude(), 0.0);
        assertEquals(lon2, bd2.getEndLongitude(), 0.0);
        assertEquals(data.getS12(), bd2.getDistanceMeters(), 0.0);
        assertEquals(data.getAzi1(), bd2.getInitialBearing(), 0.0);
        assertEquals(data.getAzi2(), bd2.getFinalBearing(), 0.0);
    }

    @Test
    public void testDistanceAndBearingArray() {
        // define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        // 41.382643,2.176700
        // 41.382524,2.176861

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        final GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        final double[] result1 = new double[1];
        final double[] result2 = new double[2];
        final double[] result3 = new double[3];

        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result1);
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result2);
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result3);

        // check
        assertEquals(data.getS12(), result1[0], 0.0);
        assertEquals(data.getS12(), result2[0], 0.0);
        assertEquals(data.getS12(), result3[0], 0.0);

        assertEquals(data.getAzi1(), result2[1], 0.0);
        assertEquals(data.getAzi1(), result3[1], 0.0);

        assertEquals(data.getAzi2(), result3[2], 0.0);

        // force IllegalArgumentException
        try {
            LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testDistanceBetween() {
        // define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        // 41.382643,2.176700
        // 41.382524,2.176861

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        final GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        assertEquals(data.getS12(), LocationUtils.distanceBetweenMeters(lat1, lon1, lat2, lon2), 0.0);

        final Distance d1 = LocationUtils.distanceBetween(lat1, lon1, lat2, lon2);
        assertEquals(data.getS12(), d1.getValue());
        assertEquals(DistanceUnit.METER, d1.getUnit());

        final Distance d2 = new Distance(0.0, DistanceUnit.MILLIMETER);
        assertSame(d2, LocationUtils.distanceBetween(lat1, lon1, lat2, lon2, d2));
        assertEquals(data.getS12(), d2.getValue());
        assertEquals(DistanceUnit.METER, d2.getUnit());
    }

    @Test
    public void testBearingDistanceConstructor() {
        final LocationUtils.BearingDistance bd = new LocationUtils.BearingDistance();

        // check default value
        assertEquals(0.0, bd.getStartLatitude(), 0.0);
        assertEquals(0.0, bd.getStartLongitude(), 0.0);
        assertEquals(0.0, bd.getEndLatitude(), 0.0);
        assertEquals(0.0, bd.getEndLongitude(), 0.0);
        assertEquals(0.0, bd.getDistanceMeters(), 0.0);

        final Distance d1 = bd.getDistance();
        assertEquals(0.0, d1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, d1.getUnit());

        final Distance d2 = new Distance(1.0, DistanceUnit.METER);
        assertSame(d2, bd.getDistance(d2));
        assertEquals(0.0, d2.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, d2.getUnit());

        assertEquals(0.0, bd.getInitialBearing(), 0.0);
        assertEquals(0.0, bd.getFinalBearing(), 0.0);
    }
}
