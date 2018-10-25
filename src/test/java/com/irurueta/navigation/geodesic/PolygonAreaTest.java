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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class PolygonAreaTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    public PolygonAreaTest() { }

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
        //test with polyline
        PolygonArea area = new PolygonArea(Geodesic.WGS84, true);

        //check
        assertEquals(area.getMajorRadius(), Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(area.getFlattening(), Constants.EARTH_FLATTENING_WGS84, 0.0);

        Pair p = area.getCurrentPoint();
        assertEquals(p.first, Double.NaN, 0.0);
        assertEquals(p.second, Double.NaN, 0.0);

        //test without polyline
        area = new PolygonArea(Geodesic.WGS84, false);

        //check
        assertEquals(area.getMajorRadius(), Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(area.getFlattening(), Constants.EARTH_FLATTENING_WGS84, 0.0);

        p = area.getCurrentPoint();
        assertEquals(p.first, Double.NaN, 0.0);
        assertEquals(p.second, Double.NaN, 0.0);
    }

    @Test
    public void testClear() {
        PolygonArea area = new PolygonArea(Geodesic.WGS84, true);

        area.clear();

        //check
        assertEquals(area.getMajorRadius(), Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(area.getFlattening(), Constants.EARTH_FLATTENING_WGS84, 0.0);

        Pair p = area.getCurrentPoint();
        assertEquals(p.first, Double.NaN, 0.0);
        assertEquals(p.second, Double.NaN, 0.0);
    }

    @Test
    public void testAddPointAndCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        double lat1 = 41.382643;
        double lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        double lat2 = 41.382524;
        double lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        double lat3 = 41.382790;
        double lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        double lat4 = 41.382911;
        double lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        PolygonResult polyResult = polyArea.compute();
        PolygonResult areaResult = area.compute();

        assertEquals(polyResult.num, 5);
        assertEquals(polyResult.perimeter, 121.34, 1.0);
        assertEquals(polyResult.area, Double.NaN, 0.0);

        assertEquals(areaResult.num, 5);
        assertEquals(areaResult.perimeter, 121.34, 1.0);
        assertEquals(areaResult.area, 815.72, 1.0);
    }

    @Test
    public void testAddEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        double lat3 = 41.382790;
        double lon3 = 2.177210;

        double lat4 = 41.382911;
        double lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        GeodesicData data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        GeodesicData data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        GeodesicData data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        PolygonArea polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        PolygonArea areaEdge = new PolygonArea(Geodesic.WGS84, false);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);
        polyAreaEdge.addPoint(lat1, lon1);
        areaEdge.addPoint(lat1, lon1);


        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);
        polyAreaEdge.addEdge(data1.azi1, data1.s12);
        areaEdge.addEdge(data1.azi1, data1.s12);

        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);
        polyAreaEdge.addEdge(data2.azi1, data2.s12);
        areaEdge.addEdge(data2.azi1, data2.s12);

        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);
        polyAreaEdge.addEdge(data3.azi1, data3.s12);
        areaEdge.addEdge(data3.azi1, data3.s12);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);
        polyAreaEdge.addEdge(data4.azi1, data4.s12);
        areaEdge.addEdge(data4.azi1, data4.s12);

        PolygonResult polyResult = polyArea.compute();
        PolygonResult areaResult = area.compute();

        PolygonResult polyEdgeResult = polyAreaEdge.compute();
        PolygonResult areaEdgeResult = areaEdge.compute();

        assertEquals(polyResult.num, 5);
        assertEquals(polyResult.perimeter, 121.34, 1.0);
        assertEquals(polyResult.area, Double.NaN, 0.0);

        assertEquals(areaResult.num, 5);
        assertEquals(areaResult.perimeter, 121.34, 1.0);
        assertEquals(areaResult.area, 815.72, 1.0);


        assertEquals(polyEdgeResult.num, 5);
        assertEquals(polyEdgeResult.perimeter, 121.34, 1.0);
        assertEquals(polyEdgeResult.area, Double.NaN, 0.0);

        assertEquals(areaEdgeResult.num, 5);
        assertEquals(areaEdgeResult.perimeter, 121.34, 1.0);
        assertEquals(areaEdgeResult.area, 815.72, 1.0);
    }

    @Test
    public void testCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        double lat = 41.382643;
        double lon = 2.176700;
        area.addPoint(lat, lon);

        lat = 41.382524;
        lon = 2.176861;
        area.addPoint(lat, lon);

        lat = 41.382790;
        lon = 2.177210;
        area.addPoint(lat, lon);

        lat = 41.382911;
        lon = 2.177009;
        area.addPoint(lat, lon);

        //compute reversed
        PolygonResult result = area.compute(true, true);

        assertEquals(result.perimeter, 121.34, 1.0);
        assertEquals(result.area, -815.72, 1.0);

        //compute remaining earth surface
        result = area.compute(true, false);

        assertNotNull(Geodesic.WGS84);
        double earthArea = Geodesic.WGS84.getEllipsoidArea();

        assertEquals(result.area, earthArea - 815.72, 1.0);
    }

    @Test
    public void testTestPoint() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        double lat1 = 41.382643;
        double lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        double lat2 = 41.382524;
        double lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        double lat3 = 41.382790;
        double lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        double lat4 = 41.382911;
        double lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        //test
        PolygonResult testPolyResult1 = polyArea.testPoint(lat1, lon1, false, true);
        PolygonResult testAreaResult1 = area.testPoint(lat1, lon1, false, true);

        //test reversed
        PolygonResult testPolyResult2 = polyArea.testPoint(lat1, lon1, true, true);
        PolygonResult testAreaResult2 = area.testPoint(lat1, lon1, true, true);

        //test remaining earth surface
        PolygonResult testPolyResult3 = polyArea.testPoint(lat1, lon1, true, false);
        PolygonResult testAreaResult3 = area.testPoint(lat1, lon1, true, false);

        PolygonResult testPolyResult4 = polyArea.testPoint(lat1, lon1, false, false);
        PolygonResult testAreaResult4 = area.testPoint(lat1, lon1, false, false);


        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        //test
        PolygonResult polyResult1 = polyArea.compute(false, true);
        PolygonResult areaResult1 = area.compute(false, true);

        //test reversed
        PolygonResult polyResult2 = polyArea.compute(true, true);
        PolygonResult areaResult2 = area.compute(true, true);

        //test remaining earth surface
        PolygonResult polyResult3 = polyArea.compute(true, false);
        PolygonResult areaResult3 = area.compute(true, false);

        PolygonResult polyResult4 = polyArea.compute(false, false);
        PolygonResult areaResult4 = area.compute(false, false);

        assertEquals(testPolyResult1.num, polyResult1.num);
        assertEquals(testPolyResult1.area, polyResult1.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult1.perimeter, polyResult1.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult2.num, polyResult2.num);
        assertEquals(testPolyResult2.area, polyResult2.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult2.perimeter, polyResult2.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult3.num, polyResult3.num);
        assertEquals(testPolyResult3.area, polyResult3.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult3.perimeter, polyResult3.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult4.num, polyResult4.num);
        assertEquals(testPolyResult4.area, polyResult4.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult4.perimeter, polyResult4.perimeter, ABSOLUTE_ERROR);

        assertEquals(testAreaResult1.num, areaResult1.num);
        assertEquals(testAreaResult1.area, areaResult1.area, LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult1.perimeter, areaResult1.perimeter, LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult2.num, areaResult2.num);
        assertEquals(testAreaResult2.area, areaResult2.area, LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult2.perimeter, areaResult2.perimeter, LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult3.num, areaResult3.num);
        assertEquals(testAreaResult3.area, areaResult3.area, LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult3.perimeter, areaResult3.perimeter, LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult4.num, areaResult4.num);
        assertEquals(testAreaResult4.area, areaResult4.area, LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult4.perimeter, areaResult4.perimeter, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testTestEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        double lat3 = 41.382790;
        double lon3 = 2.177210;

        double lat4 = 41.382911;
        double lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        GeodesicData data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        GeodesicData data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        GeodesicData data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        PolygonArea polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        PolygonArea areaEdge = new PolygonArea(Geodesic.WGS84, false);

        polyAreaEdge.addPoint(lat1, lon1);
        areaEdge.addPoint(lat1, lon1);

        polyAreaEdge.addEdge(data1.azi1, data1.s12);
        areaEdge.addEdge(data1.azi1, data1.s12);

        polyAreaEdge.addEdge(data2.azi1, data2.s12);
        areaEdge.addEdge(data2.azi1, data2.s12);

        polyAreaEdge.addEdge(data3.azi1, data3.s12);
        areaEdge.addEdge(data3.azi1, data3.s12);

        //test
        PolygonResult testPolyResult1 = polyAreaEdge.testEdge(data4.azi1, data4.s12, false, true);
        PolygonResult testAreaResult1 = areaEdge.testEdge(data4.azi1, data4.s12, false, true);

        //test reversed
        PolygonResult testPolyResult2 = polyAreaEdge.testEdge(data4.azi1, data4.s12, true, true);
        PolygonResult testAreaResult2 = areaEdge.testEdge(data4.azi1, data4.s12, true, true);

        //test remaining earth surface
        PolygonResult testPolyResult3 = polyAreaEdge.testEdge(data4.azi1, data4.s12, true, false);
        PolygonResult testAreaResult3 = areaEdge.testEdge(data4.azi1, data4.s12, true, false);

        PolygonResult testPolyResult4 = polyAreaEdge.testEdge(data4.azi1, data4.s12, false, false);
        PolygonResult testAreaResult4 = areaEdge.testEdge(data4.azi1, data4.s12, false, false);


        polyAreaEdge.addEdge(data4.azi1, data4.s12);
        areaEdge.addEdge(data4.azi1, data4.s12);

        //test
        PolygonResult polyEdgeResult1 = polyAreaEdge.compute(false, true);
        PolygonResult areaEdgeResult1 = areaEdge.compute(false, true);

        //test reversed
        PolygonResult polyEdgeResult2 = polyAreaEdge.compute(true, true);
        PolygonResult areaEdgeResult2 = areaEdge.compute(true, true);

        //test remaining earth surface
        PolygonResult polyEdgeResult3 = polyAreaEdge.compute(true, false);
        PolygonResult areaEdgeResult3 = areaEdge.compute(true, false);

        PolygonResult polyEdgeResult4 = polyAreaEdge.compute(false, false);
        PolygonResult areaEdgeResult4 = areaEdge.compute(false, false);

        assertEquals(testPolyResult1.num, polyEdgeResult1.num);
        assertEquals(testPolyResult1.area, polyEdgeResult1.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult1.perimeter, polyEdgeResult1.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult2.num, polyEdgeResult2.num);
        assertEquals(testPolyResult2.area, polyEdgeResult2.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult2.perimeter, polyEdgeResult2.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult3.num, polyEdgeResult3.num);
        assertEquals(testPolyResult3.area, polyEdgeResult3.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult3.perimeter, polyEdgeResult3.perimeter, ABSOLUTE_ERROR);

        assertEquals(testPolyResult4.num, polyEdgeResult4.num);
        assertEquals(testPolyResult4.area, polyEdgeResult4.area, ABSOLUTE_ERROR);
        assertEquals(testPolyResult4.perimeter, polyEdgeResult4.perimeter, ABSOLUTE_ERROR);

        assertEquals(testAreaResult1.num, areaEdgeResult1.num);
        assertEquals(testAreaResult1.area, areaEdgeResult1.area, ABSOLUTE_ERROR);
        assertEquals(testAreaResult1.perimeter, areaEdgeResult1.perimeter, ABSOLUTE_ERROR);

        assertEquals(testAreaResult2.num, areaEdgeResult2.num);
        assertEquals(testAreaResult2.area, areaEdgeResult2.area, ABSOLUTE_ERROR);
        assertEquals(testAreaResult2.perimeter, areaEdgeResult2.perimeter, ABSOLUTE_ERROR);

        assertEquals(testAreaResult3.num, areaEdgeResult3.num);
        assertEquals(testAreaResult3.area, areaEdgeResult3.area, ABSOLUTE_ERROR);
        assertEquals(testAreaResult3.perimeter, areaEdgeResult3.perimeter, ABSOLUTE_ERROR);

        assertEquals(testAreaResult4.num, areaEdgeResult4.num);
        assertEquals(testAreaResult4.area, areaEdgeResult4.area, ABSOLUTE_ERROR);
        assertEquals(testAreaResult4.perimeter, areaEdgeResult4.perimeter, ABSOLUTE_ERROR);
    }
}
