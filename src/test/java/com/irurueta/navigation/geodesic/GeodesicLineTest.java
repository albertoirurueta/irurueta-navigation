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
        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());

        //check
        assertEquals(line.getLatitude(), lat1, 0.0);
        assertEquals(line.getLongitude(), lon1, 0.0);
        assertEquals(line.getAzimuth(), data1.getAzi1(), 0.0);

        Pair p1 = GeoMath.sincosd(GeoMath.angRound(GeoMath.angNormalize(data1.getAzi1())));
        Pair p2 = line.getAzimuthCosines();

        assertEquals(p1.getFirst(), p2.getFirst(), ABSOLUTE_ERROR);
        assertEquals(p1.getSecond(), p2.getSecond(), ABSOLUTE_ERROR);

        Pair p = line.getEquatorialAzimuthCosines();
        assertEquals(line.getEquatorialAzimuth(), GeoMath.atan2d(p.getFirst(), p.getSecond()),
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

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());
        GeodesicData data1b = line.position(data1.getS12());

        assertEquals(data1.getLat1(), data1b.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1b.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1b.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1b.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1b.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1b.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1b.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1b.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1b.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1b.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1b.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1b.getAreaS12(), ABSOLUTE_ERROR);

        GeodesicData data1c = line.position(data1.getS12(), GeodesicMask.STANDARD);

        assertEquals(data1.getLat1(), data1c.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1c.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1c.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1c.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1c.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1c.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1c.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1c.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1c.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1c.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1c.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1c.getAreaS12(), ABSOLUTE_ERROR);

        GeodesicData data1d = line.position(false, data1.getS12(), GeodesicMask.STANDARD);

        assertEquals(data1.getLat1(), data1d.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1d.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1d.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1d.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1d.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1d.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1d.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1d.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1d.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1d.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1d.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1d.getAreaS12(), ABSOLUTE_ERROR);
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

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());
        GeodesicData data1b = line.arcPosition(data1.getA12());

        assertEquals(data1.getLat1(), data1b.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1b.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1b.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1b.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1b.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1b.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1b.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1b.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1b.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1b.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1b.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1b.getAreaS12(), ABSOLUTE_ERROR);

        GeodesicData data1c = line.arcPosition(data1.getA12(), GeodesicMask.STANDARD);

        assertEquals(data1.getLat1(), data1c.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1c.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1c.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1c.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1c.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1c.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1c.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1c.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1c.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1c.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1c.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1c.getAreaS12(), ABSOLUTE_ERROR);

        GeodesicData data1d = line.position(true, data1.getA12(), GeodesicMask.STANDARD);

        assertEquals(data1.getLat1(), data1d.getLat1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon1(), data1d.getLon1(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi1(), data1d.getAzi1(), ABSOLUTE_ERROR);
        assertEquals(data1.getLat2(), data1d.getLat2(), ABSOLUTE_ERROR);
        assertEquals(data1.getLon2(), data1d.getLon2(), ABSOLUTE_ERROR);
        assertEquals(data1.getAzi2(), data1d.getAzi2(), ABSOLUTE_ERROR);
        assertEquals(data1.getS12(), data1d.getS12(), ABSOLUTE_ERROR);
        assertEquals(data1.getA12(), data1d.getA12(), ABSOLUTE_ERROR);
        assertEquals(data1.getM12(), data1d.getM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM12(), data1d.getScaleM12(), ABSOLUTE_ERROR);
        assertEquals(data1.getScaleM21(), data1d.getScaleM21(), ABSOLUTE_ERROR);
        assertEquals(data1.getAreaS12(), data1d.getAreaS12(), ABSOLUTE_ERROR);
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

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());
        line.setDistance(data1.getS12());

        //check
        assertEquals(line.genDistance(false), data1.getS12(), ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.getS12(), ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.getA12(), ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.getA12(), ABSOLUTE_ERROR);
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

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());
        line.setArc(data1.getA12());

        //check
        assertEquals(line.genDistance(true), data1.getA12(), ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.getA12(), ABSOLUTE_ERROR);

        assertEquals(line.genDistance(false), data1.getS12(), ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.getS12(), ABSOLUTE_ERROR);
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

        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());
        line.genSetDistance(false, data1.getS12());

        //check
        assertEquals(line.genDistance(false), data1.getS12(), ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.getS12(), ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.getA12(), ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.getA12(), ABSOLUTE_ERROR);


        line.genSetDistance(true, data1.getA12());

        //check
        assertEquals(line.genDistance(false), data1.getS12(), ABSOLUTE_ERROR);
        assertEquals(line.getDistance(), data1.getS12(), ABSOLUTE_ERROR);

        assertEquals(line.genDistance(true), data1.getA12(), ABSOLUTE_ERROR);
        assertEquals(line.getArc(), data1.getA12(), ABSOLUTE_ERROR);
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
        GeodesicLine line = new GeodesicLine(Geodesic.WGS84, lat1, lon1, data1.getAzi1());

        assertEquals(line.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        assertTrue(line.capabilities(GeodesicMask.ALL));
        assertTrue(line.capabilities(GeodesicMask.LATITUDE));
        assertTrue(line.capabilities(GeodesicMask.AZIMUTH));
        assertTrue(line.capabilities(GeodesicMask.LONG_UNROLL));
    }
}
