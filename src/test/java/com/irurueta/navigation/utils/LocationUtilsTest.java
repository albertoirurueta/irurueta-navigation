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
import com.irurueta.navigation.units.Distance;
import com.irurueta.navigation.units.DistanceUnit;
import org.junit.*;

import static org.junit.Assert.*;

public class LocationUtilsTest {

    public LocationUtilsTest() { }

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
        LocationUtils utils = new LocationUtils();
        assertNotNull(utils);
    }

    @Test
    public void testConvertDoubleToString() {
        String str = LocationUtils.convert(45.0,
                LocationUtils.FORMAT_DEGREES);
        assertEquals(str, "45");


        str = LocationUtils.convert(45.5, LocationUtils.FORMAT_DEGREES);
        assertEquals(str, "45.5");


        str = LocationUtils.convert(45.0, LocationUtils.FORMAT_MINUTES);
        assertEquals(str, "45:0");


        str = LocationUtils.convert(45.5, LocationUtils.FORMAT_MINUTES);
        assertEquals(str, "45:30");


        str = LocationUtils.convert(45.54, LocationUtils.FORMAT_SECONDS);
        assertEquals(str, "45:32:24");

        str = LocationUtils.convert(-45.0, LocationUtils.FORMAT_DEGREES);
        assertEquals(str, "-45");

        //force IllegalArgumentException
        str = null;
        try {
            str = LocationUtils.convert(-181.0, LocationUtils.FORMAT_DEGREES);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            str = LocationUtils.convert(181.0, LocationUtils.FORMAT_DEGREES);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            str = LocationUtils.convert(45.0, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(str);
    }

    @Test
    public void testConvertStringToDouble() {
        double value = LocationUtils.convert("45");
        assertEquals(value, 45.0, 0.0);

        value = LocationUtils.convert("45.5");
        assertEquals(value, 45.5, 0.0);

        value = LocationUtils.convert("45:0");
        assertEquals(value, 45.0, 0.0);

        value = LocationUtils.convert("45:30");
        assertEquals(value, 45.5, 0.0);

        value = LocationUtils.convert("45:32:24");
        assertEquals(value, 45.54, 0.0);

        value = LocationUtils.convert("-45");
        assertEquals(value, -45.0, 0.0);

        //force NullPointerException
        value = 0.0;
        try {
            value = LocationUtils.convert(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }

        //force IllegalArgumentException
        try {
            value = LocationUtils.convert(" ");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("m");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("--1:30");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertEquals(value, 0.0, 0.0);
        try {
            value = LocationUtils.convert("181:30");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("45:-1");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("45:60");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("45:30:-1");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            value = LocationUtils.convert("45:30:60");
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertEquals(value, 0.0, 0.0);
    }

    @Test
    public void testDistanceAndBearing() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        LocationUtils.BearingDistance bd1 = new LocationUtils.BearingDistance();
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, bd1);

        LocationUtils.BearingDistance bd2 = LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2);

        //check
        assertEquals(bd1.getStartLatitude(), lat1, 0.0);
        assertEquals(bd1.getStartLongitude(), lon1, 0.0);
        assertEquals(bd1.getEndLatitude(), lat2, 0.0);
        assertEquals(bd1.getEndLongitude(), lon2, 0.0);
        assertEquals(bd1.getDistanceMeters(), data.s12, 0.0);
        assertEquals(bd1.getInitialBearing(), data.azi1, 0.0);
        assertEquals(bd1.getFinalBearing(), data.azi2, 0.0);

        assertEquals(bd2.getStartLatitude(), lat1, 0.0);
        assertEquals(bd2.getStartLongitude(), lon1, 0.0);
        assertEquals(bd2.getEndLatitude(), lat2, 0.0);
        assertEquals(bd2.getEndLongitude(), lon2, 0.0);
        assertEquals(bd2.getDistanceMeters(), data.s12, 0.0);
        assertEquals(bd2.getInitialBearing(), data.azi1, 0.0);
        assertEquals(bd2.getFinalBearing(), data.azi2, 0.0);
    }

    @Test
    public void testDistanceAndBearingArray() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        double[] result1 = new double[1];
        double[] result2 = new double[2];
        double[] result3 = new double[3];

        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result1);
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result2);
        LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, result3);

        //check
        assertEquals(result1[0], data.s12, 0.0);
        assertEquals(result2[0], data.s12, 0.0);
        assertEquals(result3[0], data.s12, 0.0);

        assertEquals(result2[1], data.azi1, 0.0);
        assertEquals(result3[1], data.azi1, 0.0);

        assertEquals(result3[2], data.azi2, 0.0);

        //force IllegalArgumentException
        try {
            LocationUtils.distanceAndBearing(lat1, lon1, lat2, lon2, new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testDistanceBetween() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        assertEquals(LocationUtils.distanceBetweenMeters(lat1, lon1, lat2, lon2),
                data.s12, 0.0);

        Distance d1 = LocationUtils.distanceBetween(lat1, lon1, lat2, lon2);
        assertEquals(d1.getValue(), data.s12);
        assertEquals(d1.getUnit(), DistanceUnit.METER);

        Distance d2 = new Distance(0.0, DistanceUnit.MILLIMETER);
        assertSame(LocationUtils.distanceBetween(lat1, lon1, lat2, lon2, d2), d2);
        assertEquals(d2.getValue(), data.s12);
        assertEquals(d2.getUnit(), DistanceUnit.METER);
    }

    @Test
    public void testBearingDistanceConstructor() {
        LocationUtils.BearingDistance bd = new LocationUtils.BearingDistance();

        //check default value
        assertEquals(bd.getStartLatitude(), 0.0, 0.0);
        assertEquals(bd.getStartLongitude(), 0.0, 0.0);
        assertEquals(bd.getEndLatitude(), 0.0, 0.0);
        assertEquals(bd.getEndLongitude(), 0.0, 0.0);
        assertEquals(bd.getDistanceMeters(), 0.0, 0.0);

        Distance d1 = bd.getDistance();
        assertEquals(d1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(d1.getUnit(), DistanceUnit.METER);

        Distance d2 = new Distance(1.0, DistanceUnit.METER);
        assertSame(bd.getDistance(d2), d2);
        assertEquals(d2.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(d2.getUnit(), DistanceUnit.METER);

        assertEquals(bd.getInitialBearing(), 0.0, 0.0);
        assertEquals(bd.getFinalBearing(), 0.0, 0.0);
    }
}
