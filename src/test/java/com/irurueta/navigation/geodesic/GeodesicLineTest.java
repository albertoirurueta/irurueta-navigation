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

import static org.junit.Assert.*;

public class GeodesicLineTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    public GeodesicLineTest() { }

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
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);

        //check
        assertEquals(line.getLatitude(), lat1, 0.0);
        assertEquals(line.getLongitude(), lon1, 0.0);
        assertEquals(line.getAzimuth(), data1.azi1, 0.0);

        Pair p1 = GeoMath.sincosd(GeoMath.angRound(GeoMath.angNormalize(data1.azi1)));
        Pair p2 = line.getAzimuthCosines();

        assertEquals(p1.first, p2.first, ABSOLUTE_ERROR);
        assertEquals(p1.second, p2.second, ABSOLUTE_ERROR);

        Pair p = line.getEquatorialAzimuthCosines();
        assertEquals(line.getEquatorialAzimuth(), GeoMath.atan2d(p.first, p.second),
                ABSOLUTE_ERROR);

        assertNotEquals(line.getEquatorialArc(), Double.NaN);
        assertEquals(line.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);
    }

    @Test
    public void testPosition() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);
        GeodesicData data1b = line.position(data1.s12);

        assertEquals(data1.lat1, data1b.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1b.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1b.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1b.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1b.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1b.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1b.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1b.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1b.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1b.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1b.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1b.S12, ABSOLUTE_ERROR);

        GeodesicData data1c = line.position(data1.s12, GeodesicMask.STANDARD);

        assertEquals(data1.lat1, data1c.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1c.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1c.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1c.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1c.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1c.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1c.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1c.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1c.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1c.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1c.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1c.S12, ABSOLUTE_ERROR);

        GeodesicData data1d = line.position(false, data1.s12, GeodesicMask.STANDARD);

        assertEquals(data1.lat1, data1d.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1d.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1d.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1d.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1d.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1d.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1d.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1d.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1d.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1d.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1d.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1d.S12, ABSOLUTE_ERROR);
    }

    @Test
    public void testArcPosition() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);
        GeodesicData data1b = line.arcPosition(data1.a12);

        assertEquals(data1.lat1, data1b.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1b.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1b.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1b.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1b.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1b.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1b.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1b.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1b.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1b.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1b.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1b.S12, ABSOLUTE_ERROR);

        GeodesicData data1c = line.arcPosition(data1.a12, GeodesicMask.STANDARD);

        assertEquals(data1.lat1, data1c.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1c.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1c.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1c.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1c.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1c.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1c.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1c.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1c.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1c.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1c.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1c.S12, ABSOLUTE_ERROR);

        GeodesicData data1d = line.position(true, data1.a12, GeodesicMask.STANDARD);

        assertEquals(data1.lat1, data1d.lat1, ABSOLUTE_ERROR);
        assertEquals(data1.lon1, data1d.lon1, ABSOLUTE_ERROR);
        assertEquals(data1.azi1, data1d.azi1, ABSOLUTE_ERROR);
        assertEquals(data1.lat2, data1d.lat2, ABSOLUTE_ERROR);
        assertEquals(data1.lon2, data1d.lon2, ABSOLUTE_ERROR);
        assertEquals(data1.azi2, data1d.azi2, ABSOLUTE_ERROR);
        assertEquals(data1.s12, data1d.s12, ABSOLUTE_ERROR);
        assertEquals(data1.a12, data1d.a12, ABSOLUTE_ERROR);
        assertEquals(data1.m12, data1d.m12, ABSOLUTE_ERROR);
        assertEquals(data1.M12, data1d.M12, ABSOLUTE_ERROR);
        assertEquals(data1.M21, data1d.M21, ABSOLUTE_ERROR);
        assertEquals(data1.S12, data1d.S12, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetDistance() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);
        line.setDistance(data1.s12);

        //check
        assertEquals(line.genDistance(false), data1.s12, ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.s12, ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.a12, ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.a12, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetArc() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);
        line.setArc(data1.a12);

        //check
        assertEquals(line.genDistance(true), data1.a12, ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.a12, ABSOLUTE_ERROR);

        assertEquals(line.genDistance(false), data1.s12, ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.s12, ABSOLUTE_ERROR);
    }

    @Test
    public void testGenSetDistance() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);
        line.genSetDistance(false, data1.s12);

        //check
        assertEquals(line.genDistance(false), data1.s12, ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.s12, ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.a12, ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.a12, ABSOLUTE_ERROR);


        line.genSetDistance(true, data1.a12);

        //check
        assertEquals(line.genDistance(false), data1.s12, ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.s12, ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.a12, ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.a12, ABSOLUTE_ERROR);
    }

    @Test
    public void testCapabilities() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.azi1);

        assertEquals(line.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        assertTrue(line.capabilities(GeodesicMask.ALL));
        assertTrue(line.capabilities(GeodesicMask.LATITUDE));
        assertTrue(line.capabilities(GeodesicMask.AZIMUTH));
        assertTrue(line.capabilities(GeodesicMask.LONG_UNROLL));
    }
}
