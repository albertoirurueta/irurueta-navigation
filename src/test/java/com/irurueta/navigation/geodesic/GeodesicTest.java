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

import static org.junit.Assert.*;

public class GeodesicTest {
    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final PolygonArea POLYGON =
            new PolygonArea(Geodesic.WGS84, false);
    private static final PolygonArea POLYLINE =
            new PolygonArea(Geodesic.WGS84, true);

    private static final double[][] TESTCASES = {
            {35.60777, -139.44815, 111.098748429560326,
                    -11.17491, -69.95921, 129.289270889708762,
                    8935244.5604818305, 80.50729714281974, 6273170.2055303837,
                    0.16606318447386067, 0.16479116945612937, 12841384694976.432},
            {55.52454, 106.05087, 22.020059880982801,
                    77.03196, 197.18234, 109.112041110671519,
                    4105086.1713924406, 36.892740690445894, 3828869.3344387607,
                    0.80076349608092607, 0.80101006984201008, 61674961290615.615},
            {-21.97856, 142.59065, -32.44456876433189,
                    41.84138, 98.56635, -41.84359951440466,
                    8394328.894657671, 75.62930491011522, 6161154.5773110616,
                    0.24816339233950381, 0.24930251203627892, -6637997720646.717},
            {-66.99028, 112.2363, 173.73491240878403,
                    -12.70631, 285.90344, 2.512956620913668,
                    11150344.2312080241, 100.278634181155759, 6289939.5670446687,
                    -0.17199490274700385, -0.17722569526345708, -121287239862139.744},
            {-17.42761, 173.34268, -159.033557661192928,
                    -15.84784, 5.93557, -20.787484651536988,
                    16076603.1631180673, 144.640108810286253, 3732902.1583877189,
                    -0.81273638700070476, -0.81299800519154474, 97825992354058.708},
            {32.84994, 48.28919, 150.492927788121982,
                    -56.28556, 202.29132, 48.113449399816759,
                    16727068.9438164461, 150.565799985466607, 3147838.1910180939,
                    -0.87334918086923126, -0.86505036767110637, -72445258525585.010},
            {6.96833, 52.74123, 92.581585386317712,
                    -7.39675, 206.17291, 90.721692165923907,
                    17102477.2496958388, 154.147366239113561, 2772035.6169917581,
                    -0.89991282520302447, -0.89986892177110739, -1311796973197.995},
            {-50.56724, -16.30485, -105.439679907590164,
                    -33.56571, -94.97412, -47.348547835650331,
                    6455670.5118668696, 58.083719495371259, 5409150.7979815838,
                    0.53053508035997263, 0.52988722644436602, 41071447902810.047},
            {-58.93002, -8.90775, 140.965397902500679,
                    -8.91104, 133.13503, 19.255429433416599,
                    11756066.0219864627, 105.755691241406877, 6151101.2270708536,
                    -0.26548622269867183, -0.27068483874510741, -86143460552774.735},
            {-68.82867, -74.28391, 93.774347763114881,
                    -50.63005, -8.36685, 34.65564085411343,
                    3956936.926063544, 35.572254987389284, 3708890.9544062657,
                    0.81443963736383502, 0.81420859815358342, -41845309450093.787},
            {-10.62672, -32.0898, -86.426713286747751,
                    5.883, -134.31681, -80.473780971034875,
                    11470869.3864563009, 103.387395634504061, 6184411.6622659713,
                    -0.23138683500430237, -0.23155097622286792, 4198803992123.548},
            {-21.76221, 166.90563, 29.319421206936428,
                    48.72884, 213.97627, 43.508671946410168,
                    9098627.3986554915, 81.963476716121964, 6299240.9166992283,
                    0.13965943368590333, 0.14152969707656796, 10024709850277.476},
            {-19.79938, -174.47484, 71.167275780171533,
                    -11.99349, -154.35109, 65.589099775199228,
                    2319004.8601169389, 20.896611684802389, 2267960.8703918325,
                    0.93427001867125849, 0.93424887135032789, -3935477535005.785},
            {-11.95887, -116.94513, 92.712619830452549,
                    4.57352, 7.16501, 78.64960934409585,
                    13834722.5801401374, 124.688684161089762, 5228093.177931598,
                    -0.56879356755666463, -0.56918731952397221, -9919582785894.853},
            {-87.85331, 85.66836, -65.120313040242748,
                    66.48646, 16.09921, -4.888658719272296,
                    17286615.3147144645, 155.58592449699137, 2635887.4729110181,
                    -0.90697975771398578, -0.91095608883042767, 42667211366919.534},
            {1.74708, 128.32011, -101.584843631173858,
                    -11.16617, 11.87109, -86.325793296437476,
                    12942901.1241347408, 116.650512484301857, 5682744.8413270572,
                    -0.44857868222697644, -0.44824490340007729, 10763055294345.653},
            {-25.72959, -144.90758, -153.647468693117198,
                    -57.70581, -269.17879, -48.343983158876487,
                    9413446.7452453107, 84.664533838404295, 6356176.6898881281,
                    0.09492245755254703, 0.09737058264766572, 74515122850712.444},
            {-41.22777, 122.32875, 14.285113402275739,
                    -7.57291, 130.37946, 10.805303085187369,
                    3812686.035106021, 34.34330804743883, 3588703.8812128856,
                    0.82605222593217889, 0.82572158200920196, -2456961531057.857},
            {11.01307, 138.25278, 79.43682622782374,
                    6.62726, 247.05981, 103.708090215522657,
                    11911190.819018408, 107.341669954114577, 6070904.722786735,
                    -0.29767608923657404, -0.29785143390252321, 17121631423099.696},
            {-29.47124, 95.14681, -163.779130441688382,
                    -27.46601, -69.15955, -15.909335945554969,
                    13487015.8381145492, 121.294026715742277, 5481428.9945736388,
                    -0.51527225545373252, -0.51556587964721788, 104679964020340.318}};

    @Test
    public void testConstructor() throws GeodesicException {
        Geodesic g = new Geodesic(1.0, 0.0);

        //check
        assertEquals(g.getMajorRadius(), 1.0, 0.0);
        assertEquals(g.getFlattening(), 0.0, 0.0);
        assertEquals(g.getEllipsoidArea(), 4 * Math.PI * 1.0, 0.0);

        g = null;
        try {
            g = new Geodesic(-1.0, 0.0);
            fail("GeodesicException expected but not thrown");
        } catch (GeodesicException ignore) { }
        try {
            g = new Geodesic(1.0, 1.2);
            fail("GeodesicException expected but not thrown");
        } catch (GeodesicException ignore) { }
        assertNull(g);
    }

    @Test
    public void testDirect() {
        int numValid = 0;
        for (double[] TESTCASE : TESTCASES) {
            double
                    lat1 = TESTCASE[0], lon1 = TESTCASE[1], azi1 = TESTCASE[2],
                    lat2 = TESTCASE[3], lon2 = TESTCASE[4], azi2 = TESTCASE[5],
                    s12 = TESTCASE[6], a12 = TESTCASE[7], m12 = TESTCASE[8],
                    M12 = TESTCASE[9], M21 = TESTCASE[10], S12 = TESTCASE[11];
            assertNotNull(Geodesic.WGS84);
            GeodesicData dir1 = Geodesic.WGS84.direct(lat1, lon1, azi1, s12,
                    GeodesicMask.ALL |
                            GeodesicMask.LONG_UNROLL);
            GeodesicData dir2 = Geodesic.WGS84.direct(lat1, lon1, azi1, s12);
            GeodesicData dir3 = Geodesic.WGS84.direct(lat1, lon1, azi1,
                    false, s12,
                    GeodesicMask.ALL |
                            GeodesicMask.LONG_UNROLL);

            assertEquals(lat2, dir1.getLat2(), 1e-13);
            assertEquals(lon2, dir1.getLon2(), 1e-13);
            assertEquals(azi2, dir1.getAzi2(), 1e-13);
            assertEquals(a12, dir1.getA12(), 1e-13);
            assertEquals(m12, dir1.getM12(), 1e-8);
            assertEquals(M12, dir1.getScaleM12(), 1e-15);
            assertEquals(M21, dir1.getScaleM21(), 1e-15);
            assertEquals(S12, dir1.getAreaS12(), 0.1);

            assertEquals(lat2, dir2.getLat2(), 1e-13);
            assertEquals(azi2, dir2.getAzi2(), 1e-13);
            assertEquals(a12, dir2.getA12(), 1e-13);

            assertEquals(lat2, dir3.getLat2(), 1e-13);
            assertEquals(lon2, dir3.getLon2(), 1e-13);
            assertEquals(azi2, dir3.getAzi2(), 1e-13);
            assertEquals(a12, dir3.getA12(), 1e-13);
            assertEquals(m12, dir3.getM12(), 1e-8);
            assertEquals(M12, dir3.getScaleM12(), 1e-15);
            assertEquals(M21, dir3.getScaleM21(), 1e-15);
            assertEquals(S12, dir3.getAreaS12(), 0.1);

            if(Math.abs(lon2 - dir2.getLon2()) > 1e-13) {
                continue;
            }
            assertEquals(lon2, dir2.getLon2(), 1e-13);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testArcDirect() {
        int numValid = 0;
        for (double[] TESTCASE : TESTCASES) {
            double
                    lat1 = TESTCASE[0], lon1 = TESTCASE[1], azi1 = TESTCASE[2],
                    lat2 = TESTCASE[3], lon2 = TESTCASE[4], azi2 = TESTCASE[5],
                    s12 = TESTCASE[6], a12 = TESTCASE[7], m12 = TESTCASE[8],
                    M12 = TESTCASE[9], M21 = TESTCASE[10], S12 = TESTCASE[11];
            assertNotNull(Geodesic.WGS84);
            GeodesicData dir1 = Geodesic.WGS84.arcDirect(lat1, lon1, azi1, a12,
                    GeodesicMask.ALL |
                            GeodesicMask.LONG_UNROLL);
            GeodesicData dir2 = Geodesic.WGS84.arcDirect(lat1, lon1, azi1, a12);
            GeodesicData dir3 = Geodesic.WGS84.direct(lat1, lon1, azi1,
                    true, a12, GeodesicMask.ALL |
                    GeodesicMask.LONG_UNROLL);

            assertEquals(lat2, dir1.getLat2(), 1e-13);
            assertEquals(lon2, dir1.getLon2(), 1e-13);
            assertEquals(azi2, dir1.getAzi2(), 1e-13);
            assertEquals(s12, dir1.getS12(), 1e-8);
            assertEquals(m12, dir1.getM12(), 1e-8);
            assertEquals(M12, dir1.getScaleM12(), 1e-15);
            assertEquals(M21, dir1.getScaleM21(), 1e-15);
            assertEquals(S12, dir1.getAreaS12(), 0.1);

            assertEquals(lat2, dir2.getLat2(), 1e-13);
            assertEquals(azi2, dir2.getAzi2(), 1e-13);
            assertEquals(s12, dir2.getS12(), 1e-8);

            assertEquals(lat2, dir3.getLat2(), 1e-13);
            assertEquals(lon2, dir3.getLon2(), 1e-13);
            assertEquals(azi2, dir3.getAzi2(), 1e-13);
            assertEquals(s12, dir3.getS12(), 1e-8);
            assertEquals(m12, dir3.getM12(), 1e-8);
            assertEquals(M12, dir3.getScaleM12(), 1e-15);
            assertEquals(M21, dir3.getScaleM21(), 1e-15);
            assertEquals(S12, dir3.getAreaS12(), 0.1);

            if(Math.abs(lon2 - dir2.getLon2()) > 1e-13) {
                continue;
            }
            assertEquals(lon2, dir2.getLon2(), 1e-13);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testDirectLine() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line1 = Geodesic.WGS84.directLine(lat1, lon1, data1.getAzi1(), data1.getS12());
        GeodesicLine line2 = Geodesic.WGS84.directLine(lat1, lon1, data1.getAzi1(), data1.getS12(),
                GeodesicMask.ALL);
        GeodesicLine line3 = Geodesic.WGS84.genDirectLine(lat1, lon1, data1.getAzi1(),
                false, data1.getS12(), GeodesicMask.ALL);

        //check
        assertEquals(line1.getLatitude(), lat1, 0.0);
        assertEquals(line1.getLongitude(), lon1, 0.0);
        assertEquals(line1.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line1.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2 = line1.position(data1.getS12());

        assertEquals(data2.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line2.getLatitude(), lat1, 0.0);
        assertEquals(line2.getLongitude(), lon1, 0.0);
        assertEquals(line2.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line2.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2b = line2.position(data1.getS12());

        assertEquals(data2b.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line3.getLatitude(), lat1, 0.0);
        assertEquals(line3.getLongitude(), lon1, 0.0);
        assertEquals(line3.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line3.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line3.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line3.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2c = line3.position(data1.getS12());

        assertEquals(data2c.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2c.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2c.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2c.getLon2(), lon2, ABSOLUTE_ERROR);
    }

    @Test
    public void testArcDirectLine() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line1 = Geodesic.WGS84.arcDirectLine(lat1, lon1, data1.getAzi1(), data1.getA12());
        GeodesicLine line2 = Geodesic.WGS84.arcDirectLine(lat1, lon1, data1.getAzi1(), data1.getA12(),
                GeodesicMask.ALL);
        GeodesicLine line3 = Geodesic.WGS84.genDirectLine(lat1, lon1, data1.getAzi1(), true, data1.getA12(),
                GeodesicMask.ALL);

        //check
        assertEquals(line1.getLatitude(), lat1, 0.0);
        assertEquals(line1.getLongitude(), lon1, 0.0);
        assertEquals(line1.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line1.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2 = line1.arcPosition(data1.getA12());

        assertEquals(data2.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line2.getLatitude(), lat1, 0.0);
        assertEquals(line2.getLongitude(), lon1, 0.0);
        assertEquals(line2.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line2.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2b = line2.arcPosition(data1.getA12());

        assertEquals(data2b.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line3.getLatitude(), lat1, 0.0);
        assertEquals(line3.getLongitude(), lon1, 0.0);
        assertEquals(line3.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line3.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line3.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line3.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2c = line3.arcPosition(data1.getA12());

        assertEquals(data2c.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2c.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2c.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2c.getLon2(), lon2, ABSOLUTE_ERROR);
    }

    @Test
    public void testInverse() {
        int numValid = 0;
        for (double[] TESTCASE : TESTCASES) {
            double
                    lat1 = TESTCASE[0], lon1 = TESTCASE[1], azi1 = TESTCASE[2],
                    lat2 = TESTCASE[3], lon2 = TESTCASE[4], azi2 = TESTCASE[5],
                    s12 = TESTCASE[6], a12 = TESTCASE[7], m12 = TESTCASE[8],
                    M12 = TESTCASE[9], M21 = TESTCASE[10], S12 = TESTCASE[11];
            assertNotNull(Geodesic.WGS84);
            GeodesicData inv1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2,
                    GeodesicMask.ALL |
                            GeodesicMask.LONG_UNROLL);
            GeodesicData inv2 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

            assertEquals(lon2, inv1.getLon2(), 1e-13);
            assertEquals(azi1, inv1.getAzi1(), 1e-13);
            assertEquals(azi2, inv1.getAzi2(), 1e-13);
            assertEquals(s12, inv1.getS12(), 1e-8);
            assertEquals(a12, inv1.getA12(), 1e-13);
            assertEquals(m12, inv1.getM12(), 1e-8);
            assertEquals(M12, inv1.getScaleM12(), 1e-15);
            assertEquals(M21, inv1.getScaleM21(), 1e-15);
            assertEquals(S12, inv1.getAreaS12(), 0.1);

            assertEquals(azi1, inv2.getAzi1(), 1e-13);
            assertEquals(azi2, inv2.getAzi2(), 1e-13);
            assertEquals(s12, inv2.getS12(), 1e-8);
            assertEquals(a12, inv2.getA12(), 1e-13);

            if(Math.abs(lon2 - inv2.getLon2()) > 1e-13) {
                continue;
            }
            assertEquals(lon2, inv2.getLon2(), 1e-13);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testInverseLine() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line1 = Geodesic.WGS84.inverseLine(lat1, lon1, lat2, lon2);
        GeodesicLine line2 = Geodesic.WGS84.inverseLine(lat1, lon1, lat2, lon2,
                GeodesicMask.ALL);

        //check
        assertEquals(line1.getLatitude(), lat1, 0.0);
        assertEquals(line1.getLongitude(), lon1, 0.0);
        assertEquals(line1.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line1.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2 = line1.position(data1.getS12());

        assertEquals(data2.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line2.getLatitude(), lat1, 0.0);
        assertEquals(line2.getLongitude(), lon1, 0.0);
        assertEquals(line2.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line2.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2b = line2.position(data1.getS12());

        assertEquals(data2b.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon2(), lon2, ABSOLUTE_ERROR);
    }

    @Test
    public void testLine() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);

        GeodesicLine line1 = Geodesic.WGS84.line(lat1, lon1, data1.getAzi1());
        GeodesicLine line2 = Geodesic.WGS84.line(lat1, lon1, data1.getAzi1(),
                GeodesicMask.ALL);

        //check
        assertEquals(line1.getLatitude(), lat1, 0.0);
        assertEquals(line1.getLongitude(), lon1, 0.0);
        assertEquals(line1.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line1.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line1.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2 = line1.arcPosition(data1.getA12());

        assertEquals(data2.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2.getLon2(), lon2, ABSOLUTE_ERROR);


        assertEquals(line2.getLatitude(), lat1, 0.0);
        assertEquals(line2.getLongitude(), lon1, 0.0);
        assertEquals(line2.getAzimuth(), data1.getAzi2(), 1e-3);
        assertEquals(line2.getMajorRadius(), Geodesic.WGS84.getMajorRadius(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getFlattening(), Geodesic.WGS84.getFlattening(),
                ABSOLUTE_ERROR);
        assertEquals(line2.getCapabilities(), GeodesicMask.ALL |
                GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL);

        GeodesicData data2b = line2.arcPosition(data1.getA12());

        assertEquals(data2b.getLat1(), lat1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon1(), lon1, ABSOLUTE_ERROR);
        assertEquals(data2b.getLat2(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2b.getLon2(), lon2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGeodSolve0() {
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(40.6, -73.8,
                49.01666667, 2.55);
        assertEquals(inv.getAzi1(), 53.47022, 0.5e-5);
        assertEquals(inv.getAzi2(), 111.59367, 0.5e-5);
        assertEquals(inv.getS12(), 5853226, 0.5);
    }

    @Test
    public void testGeodSolve1() {
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir = Geodesic.WGS84.direct(40.63972222, -73.77888889,
                53.5, 5850e3);
        assertEquals(dir.getLat2(), 49.01467, 0.5e-5);
        assertEquals(dir.getLon2(), 2.56106, 0.5e-5);
        assertEquals(dir.getAzi2(), 111.62947, 0.5e-5);
    }

    @Test
    public void testGeodSolve2() throws GeodesicException {
        // Check fix for antipodal prolate bug found 2010-09-04
        Geodesic geod = new Geodesic(6.4e6, -1/150.0);
        GeodesicData inv = geod.inverse(0.07476, 0, -0.07476, 180);
        assertEquals(inv.getAzi1(), 90.00078, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00078, 0.5e-5);
        assertEquals(inv.getS12(), 20106193, 0.5);
        inv = geod.inverse(0.1, 0, -0.1, 180);
        assertEquals(inv.getAzi1(), 90.00105, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00105, 0.5e-5);
        assertEquals(inv.getS12(), 20106193, 0.5);
    }

    @Test
    public void testGeodSolve4() {
        // Check fix for short line bug found 2010-05-21
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(36.493349428792, 0,
                36.49334942879201, .0000008);
        assertEquals(inv.getS12(), 0.072, 0.5e-3);
    }

    @Test
    public void testGeodSolve5() {
        // Check fix for point2=pole bug found 2010-05-03
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir = Geodesic.WGS84.direct(0.01777745589997, 30, 0, 10e6);
        assertEquals(dir.getLat2(), 90, 0.5e-5);
        if (dir.getLon2() < 0) {
            assertEquals(dir.getLon2(), -150, 0.5e-5);
            assertEquals(Math.abs(dir.getAzi2()), 180, 0.5e-5);
        } else {
            assertEquals(dir.getLon2(), 30, 0.5e-5);
            assertEquals(dir.getAzi2(), 0, 0.5e-5);
        }
    }

    @Test
    public void testGeodSolve6() {
        // Check fix for volatile sbet12a bug found 2011-06-25 (gcc 4.4.4
        // x86 -O3).  Found again on 2012-03-27 with tdm-mingw32 (g++ 4.6.1).
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv =
                Geodesic.WGS84.inverse(88.202499451857, 0,
                        -88.202499451857, 179.981022032992859592);
        assertEquals(inv.getS12(), 20003898.214, 0.5e-3);
        inv = Geodesic.WGS84.inverse(89.262080389218, 0,
                -89.262080389218, 179.992207982775375662);
        assertEquals(inv.getS12(), 20003925.854, 0.5e-3);
        inv = Geodesic.WGS84.inverse(89.333123580033, 0, -89.333123580032997687,
                179.99295812360148422);
        assertEquals(inv.getS12(), 20003926.881, 0.5e-3);
    }

    @Test
    public void testGeodSolve9() {
        // Check fix for volatile x bug found 2011-06-25 (gcc 4.4.4 x86 -O3)
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv =
                Geodesic.WGS84.inverse(56.320923501171, 0,
                        -56.320923501171, 179.664747671772880215);
        assertEquals(inv.getS12(), 19993558.287, 0.5e-3);
    }

    @Test
    public void testGeodSolve10() {
        // Check fix for adjust tol1_ bug found 2011-06-25 (Visual Studio
        // 10 rel + debug)
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv =
                Geodesic.WGS84.inverse(52.784459512564, 0,
                        -52.784459512563990912, 179.634407464943777557);
        assertEquals(inv.getS12(), 19991596.095, 0.5e-3);
    }

    @Test
    public void testGeodSolve11() {
        // Check fix for bet2 = -bet1 bug found 2011-06-25 (Visual Studio
        // 10 rel + debug)
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv =
                Geodesic.WGS84.inverse(48.522876735459, 0,
                        -48.52287673545898293, 179.599720456223079643);
        assertEquals(inv.getS12(), 19989144.774, 0.5e-3);
    }

    @Test
    public void testGeodSolve12() throws GeodesicException {
        // Check fix for inverse geodesics on extreme prolate/oblate
        // ellipsoids Reported 2012-08-29 Stefan Guenther
        // <stefan.gunther@embl.de>; fixed 2012-10-07
        Geodesic geod = new Geodesic(89.8, -1.83);
        GeodesicData inv = geod.inverse(0, 0, -10, 160);
        assertEquals(inv.getAzi1(), 120.27, 1e-2);
        assertEquals(inv.getAzi2(), 105.15, 1e-2);
        assertEquals(inv.getS12(), 266.7, 1e-1);
    }

    @Test
    public void testGeodSolve14() {
        // Check fix for inverse ignoring lon12 = nan
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(0, 0, 1, Double.NaN);
        assertTrue(isNaN(inv.getAzi1()));
        assertTrue(isNaN(inv.getAzi2()));
        assertTrue(isNaN(inv.getS12()));
    }

    @Test
    public void testGeodSolve15() throws GeodesicException {
        // Initial implementation of Math::eatanhe was wrong for e^2 < 0.  This
        // checks that this is fixed.
        Geodesic geod = new Geodesic(6.4e6, -1/150.0);
        GeodesicData dir = geod.direct(1, 2, 3, 4, GeodesicMask.AREA);
        assertEquals(dir.getAreaS12(), 23700, 0.5);
    }

    @Test
    public void testGeodSolve17() {
        // Check fix for LONG_UNROLL bug found on 2015-05-07
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir =
                Geodesic.WGS84.direct(40, -75, -10, 2e7,
                        GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), -39, 1);
        assertEquals(dir.getLon2(), -254, 1);
        assertEquals(dir.getAzi2(), -170, 1);
        GeodesicLine line = Geodesic.WGS84.line(40, -75, -10);
        dir = line.position(2e7, GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), -39, 1);
        assertEquals(dir.getLon2(), -254, 1);
        assertEquals(dir.getAzi2(), -170, 1);
        dir = Geodesic.WGS84.direct(40, -75, -10, 2e7);
        assertEquals(dir.getLat2(), -39, 1);
        assertEquals(dir.getLon2(), 105, 1);
        assertEquals(dir.getAzi2(), -170, 1);
        dir = line.position(2e7);
        assertEquals(dir.getLat2(), -39, 1);
        assertEquals(dir.getLon2(), 105, 1);
        assertEquals(dir.getAzi2(), -170, 1);
    }

    @Test
    public void testGeodSolve26() throws GeodesicException {
        // Check 0/0 problem with area calculation on sphere 2015-09-08
        Geodesic geod = new Geodesic(6.4e6, 0);
        GeodesicData inv = geod.inverse(1, 2, 3, 4, GeodesicMask.AREA);
        assertEquals(inv.getAreaS12(), 49911046115.0, 0.5);
    }

    @Test
    public void testGeodSolve28() throws GeodesicException {
        // Check for bad placement of assignment of r.a12 with |f| > 0.01 (bug in
        // Java implementation fixed on 2015-05-19).
        Geodesic geod = new Geodesic(6.4e6, 0.1);
        GeodesicData dir = geod.direct(1, 2, 10, 5e6);
        assertEquals(dir.getA12(), 48.55570690, 0.5e-8);
    }

    @Test
    public void testGeodSolve29() {
        // Check longitude unrolling with inverse calculation 2015-09-16
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir = Geodesic.WGS84.inverse(0, 539, 0, 181);
        assertEquals(dir.getLon1(), 179, 1e-10);
        assertEquals(dir.getLon2(), -179, 1e-10);
        assertEquals(dir.getS12(), 222639, 0.5);
        dir = Geodesic.WGS84.inverse(0, 539, 0, 181,
                GeodesicMask.STANDARD |
                        GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLon1(), 539, 1e-10);
        assertEquals(dir.getLon2(), 541, 1e-10);
        assertEquals(dir.getS12(), 222639, 0.5);
    }

    @Test
    public void testGeodSolve33() throws GeodesicException {
        // Check max(-0.0,+0.0) issues 2015-08-22 (triggered by bugs in Octave --
        // sind(-0.0) = +0.0 -- and in some version of Visual Studio --
        // fmod(-0.0, 360.0) = +0.0.
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(0, 0, 0, 179);
        assertEquals(inv.getAzi1(), 90.00000, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00000, 0.5e-5);
        assertEquals(inv.getS12(), 19926189, 0.5);
        inv = Geodesic.WGS84.inverse(0, 0, 0, 179.5);
        assertEquals(inv.getAzi1(), 55.96650, 0.5e-5);
        assertEquals(inv.getAzi2(), 124.03350, 0.5e-5);
        assertEquals(inv.getS12(), 19980862, 0.5);
        inv = Geodesic.WGS84.inverse(0, 0, 0, 180);
        assertEquals(inv.getAzi1(), 0.00000, 0.5e-5);
        assertEquals(Math.abs(inv.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(inv.getS12(), 20003931, 0.5);
        inv = Geodesic.WGS84.inverse(0, 0, 1, 180);
        assertEquals(inv.getAzi1(), 0.00000, 0.5e-5);
        assertEquals(Math.abs(inv.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(inv.getS12(), 19893357, 0.5);
        Geodesic geod = new Geodesic(6.4e6, 0);
        inv = geod.inverse(0, 0, 0, 179);
        assertEquals(inv.getAzi1(), 90.00000, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00000, 0.5e-5);
        assertEquals(inv.getS12(), 19994492, 0.5);
        inv = geod.inverse(0, 0, 0, 180);
        assertEquals(inv.getAzi1(), 0.00000, 0.5e-5);
        assertEquals(Math.abs(inv.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(inv.getS12(), 20106193, 0.5);
        inv = geod.inverse(0, 0, 1, 180);
        assertEquals(inv.getAzi1(), 0.00000, 0.5e-5);
        assertEquals(Math.abs(inv.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(inv.getS12(), 19994492, 0.5);
        geod = new Geodesic(6.4e6, -1/300.0);
        inv = geod.inverse(0, 0, 0, 179);
        assertEquals(inv.getAzi1(), 90.00000, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00000, 0.5e-5);
        assertEquals(inv.getS12(), 19994492, 0.5);
        inv = geod.inverse(0, 0, 0, 180);
        assertEquals(inv.getAzi1(), 90.00000, 0.5e-5);
        assertEquals(inv.getAzi2(), 90.00000, 0.5e-5);
        assertEquals(inv.getS12(), 20106193, 0.5);
        inv = geod.inverse(0, 0, 0.5, 180);
        assertEquals(inv.getAzi1(), 33.02493, 0.5e-5);
        assertEquals(inv.getAzi2(), 146.97364, 0.5e-5);
        assertEquals(inv.getS12(), 20082617, 0.5);
        inv = geod.inverse(0, 0, 1, 180);
        assertEquals(inv.getAzi1(), 0.00000, 0.5e-5);
        assertEquals(Math.abs(inv.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(inv.getS12(), 20027270, 0.5);
    }

    @Test
    public void testGeodSolve55() {
        // Check fix for nan + point on equator or pole not returning all nans in
        // Geodesic::Inverse, found 2015-09-23.
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(Double.NaN, 0, 0, 90);
        assertTrue(isNaN(inv.getAzi1()));
        assertTrue(isNaN(inv.getAzi2()));
        assertTrue(isNaN(inv.getS12()));
        inv = Geodesic.WGS84.inverse(Double.NaN, 0, 90, 3);
        assertTrue(isNaN(inv.getAzi1()));
        assertTrue(isNaN(inv.getAzi2()));
        assertTrue(isNaN(inv.getS12()));
    }

    @Test
    public void testGeodSolve59() {
        // Check for points close with longitudes close to 180 deg apart.
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(5, 0.00000000000001, 10, 180);
        assertEquals(inv.getAzi1(), 0.000000000000035, 1.5e-14);
        assertEquals(inv.getAzi2(), 179.99999999999996, 1.5e-14);
        assertEquals(inv.getS12(), 18345191.174332713, 4e-9);
    }

    @Test
    public void testGeodSolve61() {
        // Make sure small negative azimuths are west-going
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir =
                Geodesic.WGS84.direct(45, 0, -0.000000000000000003, 1e7,
                        GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), 45.30632, 0.5e-5);
        assertEquals(dir.getLon2(), -180, 0.5e-5);
        assertEquals(Math.abs(dir.getAzi2()), 180, 0.5e-5);
        GeodesicLine line = Geodesic.WGS84.inverseLine(45, 0, 80,
                -0.000000000000000003);
        dir = line.position(1e7, GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), 45.30632, 0.5e-5);
        assertEquals(dir.getLon2(), -180, 0.5e-5);
        assertEquals(Math.abs(dir.getAzi2()), 180, 0.5e-5);
    }

    @Test
    public void testGeodSolve65() {
        // Check for bug in east-going check in GeodesicLine (needed to check for
        // sign of 0) and sign error in area calculation due to a bogus override
        // of the code for alp12.  Found/fixed on 2015-12-19.
        assertNotNull(Geodesic.WGS84);
        GeodesicLine line = Geodesic.WGS84.inverseLine(30, -0.000000000000000001,
                -31, 180);
        GeodesicData dir =
                line.position(1e7, GeodesicMask.ALL | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat1(), 30.00000  , 0.5e-5);
        assertEquals(dir.getLon1(), -0.00000  , 0.5e-5);
        assertEquals(Math.abs(dir.getAzi1()), 180.00000, 0.5e-5);
        assertEquals(dir.getLat2(), -60.23169 , 0.5e-5);
        assertEquals(dir.getLon2(), -0.00000  , 0.5e-5);
        assertEquals(Math.abs(dir.getAzi2()), 180.00000, 0.5e-5);
        assertEquals(dir.getS12(), 10000000  , 0.5);
        assertEquals(dir.getA12() , 90.06544  , 0.5e-5);
        assertEquals(dir.getM12() , 6363636   , 0.5);
        assertEquals(dir.getScaleM12() , -0.0012834, 0.5e7);
        assertEquals(dir.getScaleM21() , 0.0013749 , 0.5e-7);
        assertEquals(dir.getAreaS12() , 0         , 0.5);
        dir = line.position(2e7, GeodesicMask.ALL | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat1(), 30.00000  , 0.5e-5);
        assertEquals(dir.getLon1(), -0.00000  , 0.5e-5);
        assertEquals(Math.abs(dir.getAzi1()), 180.00000, 0.5e-5);
        assertEquals(dir.getLat2(), -30.03547 , 0.5e-5);
        assertEquals(dir.getLon2(), -180.00000, 0.5e-5);
        assertEquals(dir.getAzi2(), -0.00000  , 0.5e-5);
        assertEquals(dir.getS12() , 20000000  , 0.5);
        assertEquals(dir.getA12() , 179.96459 , 0.5e-5);
        assertEquals(dir.getM12() , 54342     , 0.5);
        assertEquals(dir.getScaleM12() , -1.0045592, 0.5e7);
        assertEquals(dir.getScaleM21() , -0.9954339, 0.5e-7);
        assertEquals(dir.getAreaS12() , 127516405431022.0, 0.5);
    }

    @Test
    public void testGeodSolve69() {
        // Check for InverseLine if line is slightly west of S and that s13 is
        // correctly set.
        assertNotNull(Geodesic.WGS84);
        GeodesicLine line =
                Geodesic.WGS84.inverseLine(-5, -0.000000000000002, -10, 180);
        GeodesicData dir =
                line.position(2e7, GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), 4.96445   , 0.5e-5);
        assertEquals(dir.getLon2(), -180.00000, 0.5e-5);
        assertEquals(dir.getAzi2(), -0.00000  , 0.5e-5);
        dir = line.position(0.5 * line.getDistance(),
                GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), -87.52461 , 0.5e-5);
        assertEquals(dir.getLon2(), -0.00000  , 0.5e-5);
        assertEquals(dir.getAzi2(), -180.00000, 0.5e-5);
    }

    @Test
    public void testGeodSolve71() {
        // Check that DirectLine sets s13.
        assertNotNull(Geodesic.WGS84);
        GeodesicLine line = Geodesic.WGS84.directLine(1, 2, 45, 1e7);
        GeodesicData dir =
                line.position(0.5 * line.getDistance(),
                        GeodesicMask.STANDARD | GeodesicMask.LONG_UNROLL);
        assertEquals(dir.getLat2(), 30.92625, 0.5e-5);
        assertEquals(dir.getLon2(), 37.54640, 0.5e-5);
        assertEquals(dir.getAzi2(), 55.43104, 0.5e-5);
    }

    @Test
    public void testGeodSolve73() {
        // Check for backwards from the pole bug reported by Anon on 2016-02-13.
        // This only affected the Java implementation.  It was introduced in Java
        // version 1.44 and fixed in 1.46-SNAPSHOT on 2016-01-17.
        assertNotNull(Geodesic.WGS84);
        GeodesicData dir = Geodesic.WGS84.direct(90, 10, 180, -1e6);
        assertEquals(dir.getLat2(), 81.04623, 0.5e-5);
        assertEquals(dir.getLon2(), -170, 0.5e-5);
        assertEquals(dir.getAzi2(), 0, 0.5e-5);
    }

    @Test
    public void testGeodSolve74() {
        // Check fix for inaccurate areas, bug introduced in v1.46, fixed
        // 2015-10-16.
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(54.1589, 15.3872,
                54.1591, 15.3877,
                GeodesicMask.ALL);
        assertEquals(inv.getAzi1(), 55.723110355, 5e-9);
        assertEquals(inv.getAzi2(), 55.723515675, 5e-9);
        assertEquals(inv.getS12(),  39.527686385, 5e-9);
        assertEquals(inv.getA12(),   0.000355495, 5e-9);
        assertEquals(inv.getM12(),  39.527686385, 5e-9);
        assertEquals(inv.getScaleM12(),   0.999999995, 5e-9);
        assertEquals(inv.getScaleM21(),   0.999999995, 5e-9);
        assertEquals(inv.getAreaS12(), 286698586.30197, 5e-4);
    }

    @Test
    public void testGeodSolve76() {
        // The distance from Wellington and Salamanca (a classic failure of
        // Vincenty)
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(-(41+19/60.0), 174+49/60.0,
                40+58/60.0, -(5+30/60.0));
        assertEquals(inv.getAzi1(), 160.39137649664, 0.5e-11);
        assertEquals(inv.getAzi2(),  19.50042925176, 0.5e-11);
        assertEquals(inv.getS12(),  19960543.857179, 0.5e-6);
    }

    @Test
    public void testGeodSolve78() {
        // An example where the NGS calculator fails to converge
        assertNotNull(Geodesic.WGS84);
        GeodesicData inv = Geodesic.WGS84.inverse(27.2, 0.0, -27.1, 179.5);
        assertEquals(inv.getAzi1(),  45.82468716758, 0.5e-11);
        assertEquals(inv.getAzi2(), 134.22776532670, 0.5e-11);
        assertEquals(inv.getS12(),  19974354.765767, 0.5e-6);
    }

    @Test
    public void testPlanimeter0() {
        // Check fix for pole-encircling bug found 2011-03-16
        double[][] pa = {{89, 0}, {89, 90}, {89, 180}, {89, 270}};
        PolygonResult a = Planimeter(pa);
        assertEquals(a.getPerimeter(), 631819.8745, 1e-4);
        assertEquals(a.getArea(), 24952305678.0, 1);

        double[][] pb = {{-89, 0}, {-89, 90}, {-89, 180}, {-89, 270}};
        a = Planimeter(pb);
        assertEquals(a.getPerimeter(), 631819.8745, 1e-4);
        assertEquals(a.getArea(), -24952305678.0, 1);

        double[][] pc = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
        a = Planimeter(pc);
        assertEquals(a.getPerimeter(), 627598.2731, 1e-4);
        assertEquals(a.getArea(), 24619419146.0, 1);

        double[][] pd = {{90, 0}, {0, 0}, {0, 90}};
        a = Planimeter(pd);
        assertEquals(a.getPerimeter(), 30022685, 1);
        assertEquals(a.getArea(), 63758202715511.0, 1);
        a = polyLength(pd);
        assertEquals(a.getPerimeter(), 20020719, 1);
        assertTrue(isNaN(a.getArea()));
    }

    @Test
    public void testPlanimeter5() {
        // Check fix for Planimeter pole crossing bug found 2011-06-24
        double[][] points = {{89, 0.1}, {89, 90.1}, {89, -179.9}};
        PolygonResult a = Planimeter(points);
        assertEquals(a.getPerimeter(), 539297, 1);
        assertEquals(a.getArea(), 12476152838.5, 1);
    }

    @Test
    public void testPlanimeter6() {
        // Check fix for Planimeter lon12 rounding bug found 2012-12-03
        double[][] pa = {{9, -0.00000000000001}, {9, 180}, {9, 0}};
        PolygonResult a = Planimeter(pa);
        assertEquals(a.getPerimeter(), 36026861, 1);
        assertEquals(a.getArea(), 0, 1);
        double[][] pb = {{9, 0.00000000000001}, {9, 0}, {9, 180}};
        a = Planimeter(pb);
        assertEquals(a.getPerimeter(), 36026861, 1);
        assertEquals(a.getArea(), 0, 1);
        double[][] pc = {{9, 0.00000000000001}, {9, 180}, {9, 0}};
        a = Planimeter(pc);
        assertEquals(a.getPerimeter(), 36026861, 1);
        assertEquals(a.getArea(), 0, 1);
        double[][] pd = {{9, -0.00000000000001}, {9, 0}, {9, 180}};
        a = Planimeter(pd);
        assertEquals(a.getPerimeter(), 36026861, 1);
        assertEquals(a.getArea(), 0, 1);
    }

    @Test
    public void testPlanimeter12() {
        // Area of arctic circle (not really -- adjunct to rhumb-area test)
        double[][] points = {{66.562222222, 0}, {66.562222222, 180}};
        PolygonResult a = Planimeter(points);
        assertEquals(a.getPerimeter(), 10465729, 1);
        assertEquals(a.getArea(), 0, 1);
    }

    @Test
    public void testPlanimeter13() {
        // Check encircling pole twice
        double[][] points = {{89, -360}, {89, -240}, {89, -120},
                {89, 0}, {89, 120}, {89, 240}};
        PolygonResult a =  Planimeter(points);
        assertEquals(a.getPerimeter(), 1160741, 1);
        assertEquals(a.getArea(), 32415230256.0, 1);
    }

    private static boolean isNaN(double x) {
        return x != x;
    }

    private static PolygonResult Planimeter(double[][] points) {
        POLYGON.clear();
        for (double[] point : points) {
            POLYGON.addPoint(point[0], point[1]);
        }
        return POLYGON.compute(false, true);
    }

    private static PolygonResult polyLength(double[][] points) {
        POLYLINE.clear();
        for (double[] point : points) {
            POLYLINE.addPoint(point[0], point[1]);
        }
        return POLYLINE.compute(false, true);
    }
}
