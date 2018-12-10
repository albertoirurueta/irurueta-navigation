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
        assertEquals(p.getFirst(), Double.NaN, 0.0);
        assertEquals(p.getSecond(), Double.NaN, 0.0);

        //test without polyline
        area = new PolygonArea(Geodesic.WGS84, false);

        //check
        assertEquals(area.getMajorRadius(), Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(area.getFlattening(), Constants.EARTH_FLATTENING_WGS84, 0.0);

        p = area.getCurrentPoint();
        assertEquals(p.getFirst(), Double.NaN, 0.0);
        assertEquals(p.getSecond(), Double.NaN, 0.0);
    }

    @Test
    public void testClear() {
        PolygonArea area = new PolygonArea(Geodesic.WGS84, true);

        area.clear();

        //check
        assertEquals(area.getMajorRadius(), Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(area.getFlattening(), Constants.EARTH_FLATTENING_WGS84, 0.0);

        Pair p = area.getCurrentPoint();
        assertEquals(p.getFirst(), Double.NaN, 0.0);
        assertEquals(p.getSecond(), Double.NaN, 0.0);
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

        assertEquals(polyResult.getNum(), 5);
        assertEquals(polyResult.getPerimeter(), 121.34, 1.0);
        assertEquals(polyResult.getArea(), Double.NaN, 0.0);

        assertEquals(areaResult.getNum(), 5);
        assertEquals(areaResult.getPerimeter(), 121.34, 1.0);
        assertEquals(areaResult.getArea(), 815.72, 1.0);
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
        polyAreaEdge.addEdge(data1.getAzi1(), data1.getS12());
        areaEdge.addEdge(data1.getAzi1(), data1.getS12());

        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);
        polyAreaEdge.addEdge(data2.getAzi1(), data2.getS12());
        areaEdge.addEdge(data2.getAzi1(), data2.getS12());

        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);
        polyAreaEdge.addEdge(data3.getAzi1(), data3.getS12());
        areaEdge.addEdge(data3.getAzi1(), data3.getS12());

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);
        polyAreaEdge.addEdge(data4.getAzi1(), data4.getS12());
        areaEdge.addEdge(data4.getAzi1(), data4.getS12());

        PolygonResult polyResult = polyArea.compute();
        PolygonResult areaResult = area.compute();

        PolygonResult polyEdgeResult = polyAreaEdge.compute();
        PolygonResult areaEdgeResult = areaEdge.compute();

        assertEquals(polyResult.getNum(), 5);
        assertEquals(polyResult.getPerimeter(), 121.34, 1.0);
        assertEquals(polyResult.getArea(), Double.NaN, 0.0);

        assertEquals(areaResult.getNum(), 5);
        assertEquals(areaResult.getPerimeter(), 121.34, 1.0);
        assertEquals(areaResult.getArea(), 815.72, 1.0);


        assertEquals(polyEdgeResult.getNum(), 5);
        assertEquals(polyEdgeResult.getPerimeter(), 121.34, 1.0);
        assertEquals(polyEdgeResult.getArea(), Double.NaN, 0.0);

        assertEquals(areaEdgeResult.getNum(), 5);
        assertEquals(areaEdgeResult.getPerimeter(), 121.34, 1.0);
        assertEquals(areaEdgeResult.getArea(), 815.72, 1.0);
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

        assertEquals(result.getPerimeter(), 121.34, 1.0);
        assertEquals(result.getArea(), -815.72, 1.0);

        //compute remaining earth surface
        result = area.compute(true, false);

        assertNotNull(Geodesic.WGS84);
        double earthArea = Geodesic.WGS84.getEllipsoidArea();

        assertEquals(result.getArea(), earthArea - 815.72, 1.0);
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

        assertEquals(testPolyResult1.getNum(), polyResult1.getNum());
        assertEquals(testPolyResult1.getArea(), polyResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult1.getPerimeter(), polyResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult2.getNum(), polyResult2.getNum());
        assertEquals(testPolyResult2.getArea(), polyResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult2.getPerimeter(), polyResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult3.getNum(), polyResult3.getNum());
        assertEquals(testPolyResult3.getArea(), polyResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult3.getPerimeter(), polyResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult4.getNum(), polyResult4.getNum());
        assertEquals(testPolyResult4.getArea(), polyResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult4.getPerimeter(), polyResult4.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testAreaResult1.getNum(), areaResult1.getNum());
        assertEquals(testAreaResult1.getArea(), areaResult1.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult1.getPerimeter(), areaResult1.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult2.getNum(), areaResult2.getNum());
        assertEquals(testAreaResult2.getArea(), areaResult2.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult2.getPerimeter(), areaResult2.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult3.getNum(), areaResult3.getNum());
        assertEquals(testAreaResult3.getArea(), areaResult3.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult3.getPerimeter(), areaResult3.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(testAreaResult4.getNum(), areaResult4.getNum());
        assertEquals(testAreaResult4.getArea(), areaResult4.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(testAreaResult4.getPerimeter(), areaResult4.getPerimeter(), LARGE_ABSOLUTE_ERROR);
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

        polyAreaEdge.addEdge(data1.getAzi1(), data1.getS12());
        areaEdge.addEdge(data1.getAzi1(), data1.getS12());

        polyAreaEdge.addEdge(data2.getAzi1(), data2.getS12());
        areaEdge.addEdge(data2.getAzi1(), data2.getS12());

        polyAreaEdge.addEdge(data3.getAzi1(), data3.getS12());
        areaEdge.addEdge(data3.getAzi1(), data3.getS12());

        //test
        PolygonResult testPolyResult1 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false,
                true);
        PolygonResult testAreaResult1 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, true);

        //test reversed
        PolygonResult testPolyResult2 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true,
                true);
        PolygonResult testAreaResult2 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, true);

        //test remaining earth surface
        PolygonResult testPolyResult3 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true,
                false);
        PolygonResult testAreaResult3 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, false);

        PolygonResult testPolyResult4 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false,
                false);
        PolygonResult testAreaResult4 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, false);


        polyAreaEdge.addEdge(data4.getAzi1(), data4.getS12());
        areaEdge.addEdge(data4.getAzi1(), data4.getS12());

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

        assertEquals(testPolyResult1.getNum(), polyEdgeResult1.getNum());
        assertEquals(testPolyResult1.getArea(), polyEdgeResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult1.getPerimeter(), polyEdgeResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult2.getNum(), polyEdgeResult2.getNum());
        assertEquals(testPolyResult2.getArea(), polyEdgeResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult2.getPerimeter(), polyEdgeResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult3.getNum(), polyEdgeResult3.getNum());
        assertEquals(testPolyResult3.getArea(), polyEdgeResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult3.getPerimeter(), polyEdgeResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testPolyResult4.getNum(), polyEdgeResult4.getNum());
        assertEquals(testPolyResult4.getArea(), polyEdgeResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(testPolyResult4.getPerimeter(), polyEdgeResult4.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testAreaResult1.getNum(), areaEdgeResult1.getNum());
        assertEquals(testAreaResult1.getArea(), areaEdgeResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(testAreaResult1.getPerimeter(), areaEdgeResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testAreaResult2.getNum(), areaEdgeResult2.getNum());
        assertEquals(testAreaResult2.getArea(), areaEdgeResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(testAreaResult2.getPerimeter(), areaEdgeResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testAreaResult3.getNum(), areaEdgeResult3.getNum());
        assertEquals(testAreaResult3.getArea(), areaEdgeResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(testAreaResult3.getPerimeter(), areaEdgeResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(testAreaResult4.getNum(), areaEdgeResult4.getNum());
        assertEquals(testAreaResult4.getArea(), areaEdgeResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(testAreaResult4.getPerimeter(), areaEdgeResult4.getPerimeter(), ABSOLUTE_ERROR);
    }
}
