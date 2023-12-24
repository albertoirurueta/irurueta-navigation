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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class PolygonAreaTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstructor() {
        //test with polyline
        PolygonArea area = new PolygonArea(Geodesic.WGS84, true);

        //check
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, area.getMajorRadius(), 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, area.getFlattening(), 0.0);

        Pair p = area.getCurrentPoint();
        assertEquals(Double.NaN, p.getFirst(), 0.0);
        assertEquals(Double.NaN, p.getSecond(), 0.0);

        //test without polyline
        area = new PolygonArea(Geodesic.WGS84, false);

        //check
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, area.getMajorRadius(), 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, area.getFlattening(), 0.0);

        p = area.getCurrentPoint();
        assertEquals(Double.NaN, p.getFirst(), 0.0);
        assertEquals(Double.NaN, p.getSecond(), 0.0);
    }

    @Test
    public void testClear() {
        final PolygonArea area = new PolygonArea(Geodesic.WGS84, true);

        area.clear();

        //check
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, area.getMajorRadius(), 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, area.getFlattening(), 0.0);

        final Pair p = area.getCurrentPoint();
        assertEquals(Double.NaN, p.getFirst(), 0.0);
        assertEquals(Double.NaN, p.getSecond(), 0.0);
    }

    @Test
    public void testAddPointAndCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        final PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        final double lat3 = 41.382790;
        final double lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        final double lat4 = 41.382911;
        final double lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        PolygonResult polyResult = polyArea.compute();
        PolygonResult areaResult = area.compute();

        assertEquals(5, polyResult.getNum());
        assertEquals(121.34, polyResult.getPerimeter(), 1.0);
        assertEquals(Double.NaN, polyResult.getArea(), 0.0);

        assertEquals(5, areaResult.getNum());
        assertEquals(121.34, areaResult.getPerimeter(), 1.0);
        assertEquals(815.72, areaResult.getArea(), 1.0);
    }

    @Test
    public void testAddEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        final double lat3 = 41.382790;
        final double lon3 = 2.177210;

        final double lat4 = 41.382911;
        final double lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        GeodesicData data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        GeodesicData data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        GeodesicData data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        final PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        final PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        final PolygonArea polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        final PolygonArea areaEdge = new PolygonArea(Geodesic.WGS84, false);

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

        final PolygonResult polyResult = polyArea.compute();
        final PolygonResult areaResult = area.compute();

        final PolygonResult polyEdgeResult = polyAreaEdge.compute();
        final PolygonResult areaEdgeResult = areaEdge.compute();

        assertEquals(5, polyResult.getNum());
        assertEquals(121.34, polyResult.getPerimeter(), 1.0);
        assertEquals(Double.NaN, polyResult.getArea(), 0.0);

        assertEquals(5, areaResult.getNum());
        assertEquals(121.34, areaResult.getPerimeter(), 1.0);
        assertEquals(815.72, areaResult.getArea(), 1.0);


        assertEquals(5, polyEdgeResult.getNum());
        assertEquals(121.34, polyEdgeResult.getPerimeter(), 1.0);
        assertEquals(Double.NaN, polyEdgeResult.getArea(), 0.0);

        assertEquals(5, areaEdgeResult.getNum());
        assertEquals(121.34, areaEdgeResult.getPerimeter(), 1.0);
        assertEquals(815.72, areaEdgeResult.getArea(), 1.0);
    }

    @Test
    public void testCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

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

        assertEquals(121.34, result.getPerimeter(), 1.0);
        assertEquals(-815.72, result.getArea(), 1.0);

        //compute remaining earth surface
        result = area.compute(true, false);

        assertNotNull(Geodesic.WGS84);
        final double earthArea = Geodesic.WGS84.getEllipsoidArea();

        assertEquals(earthArea - 815.72, result.getArea(), 1.0);
    }

    @Test
    public void testTestPoint() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final PolygonArea polyArea = new PolygonArea(Geodesic.WGS84, true);
        final PolygonArea area = new PolygonArea(Geodesic.WGS84, false);

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        final double lat3 = 41.382790;
        final double lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        final double lat4 = 41.382911;
        final double lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        //test
        final PolygonResult testPolyResult1 = polyArea.testPoint(lat1, lon1, false, true);
        final PolygonResult testAreaResult1 = area.testPoint(lat1, lon1, false, true);

        //test reversed
        final PolygonResult testPolyResult2 = polyArea.testPoint(lat1, lon1, true, true);
        final PolygonResult testAreaResult2 = area.testPoint(lat1, lon1, true, true);

        //test remaining earth surface
        final PolygonResult testPolyResult3 = polyArea.testPoint(lat1, lon1, true, false);
        final PolygonResult testAreaResult3 = area.testPoint(lat1, lon1, true, false);

        final PolygonResult testPolyResult4 = polyArea.testPoint(lat1, lon1, false, false);
        final PolygonResult testAreaResult4 = area.testPoint(lat1, lon1, false, false);


        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        //test
        final PolygonResult polyResult1 = polyArea.compute(false, true);
        final PolygonResult areaResult1 = area.compute(false, true);

        //test reversed
        final PolygonResult polyResult2 = polyArea.compute(true, true);
        final PolygonResult areaResult2 = area.compute(true, true);

        //test remaining earth surface
        final PolygonResult polyResult3 = polyArea.compute(true, false);
        final PolygonResult areaResult3 = area.compute(true, false);

        final PolygonResult polyResult4 = polyArea.compute(false, false);
        final PolygonResult areaResult4 = area.compute(false, false);

        assertEquals(polyResult1.getNum(), testPolyResult1.getNum());
        assertEquals(polyResult1.getArea(), testPolyResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyResult1.getPerimeter(), testPolyResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyResult2.getNum(), testPolyResult2.getNum());
        assertEquals(polyResult2.getArea(), testPolyResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyResult2.getPerimeter(), testPolyResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyResult3.getNum(), testPolyResult3.getNum());
        assertEquals(polyResult3.getArea(), testPolyResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyResult3.getPerimeter(), testPolyResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyResult4.getNum(), testPolyResult4.getNum());
        assertEquals(polyResult4.getArea(), testPolyResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyResult4.getPerimeter(), testPolyResult4.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(areaResult1.getNum(), testAreaResult1.getNum());
        assertEquals(areaResult1.getArea(), testAreaResult1.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(areaResult1.getPerimeter(), testAreaResult1.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(areaResult2.getNum(), testAreaResult2.getNum());
        assertEquals(areaResult2.getArea(), testAreaResult2.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(areaResult2.getPerimeter(), testAreaResult2.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(areaResult3.getNum(), testAreaResult3.getNum());
        assertEquals(areaResult3.getArea(), testAreaResult3.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(areaResult3.getPerimeter(), testAreaResult3.getPerimeter(), LARGE_ABSOLUTE_ERROR);

        assertEquals(areaResult4.getNum(), testAreaResult4.getNum());
        assertEquals(areaResult4.getArea(), testAreaResult4.getArea(), LARGE_ABSOLUTE_ERROR);
        assertEquals(areaResult4.getPerimeter(), testAreaResult4.getPerimeter(), LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testTestEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        final double lat3 = 41.382790;
        final double lon3 = 2.177210;

        final double lat4 = 41.382911;
        final double lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        final GeodesicData data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        final GeodesicData data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        final GeodesicData data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        final GeodesicData data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        final PolygonArea polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        final PolygonArea areaEdge = new PolygonArea(Geodesic.WGS84, false);

        polyAreaEdge.addPoint(lat1, lon1);
        areaEdge.addPoint(lat1, lon1);

        polyAreaEdge.addEdge(data1.getAzi1(), data1.getS12());
        areaEdge.addEdge(data1.getAzi1(), data1.getS12());

        polyAreaEdge.addEdge(data2.getAzi1(), data2.getS12());
        areaEdge.addEdge(data2.getAzi1(), data2.getS12());

        polyAreaEdge.addEdge(data3.getAzi1(), data3.getS12());
        areaEdge.addEdge(data3.getAzi1(), data3.getS12());

        //test
        final PolygonResult testPolyResult1 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false,
                true);
        final PolygonResult testAreaResult1 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, true);

        //test reversed
        final PolygonResult testPolyResult2 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true,
                true);
        final PolygonResult testAreaResult2 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, true);

        //test remaining earth surface
        final PolygonResult testPolyResult3 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true,
                false);
        final PolygonResult testAreaResult3 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, false);

        final PolygonResult testPolyResult4 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false,
                false);
        final PolygonResult testAreaResult4 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, false);


        polyAreaEdge.addEdge(data4.getAzi1(), data4.getS12());
        areaEdge.addEdge(data4.getAzi1(), data4.getS12());

        //test
        final PolygonResult polyEdgeResult1 = polyAreaEdge.compute(false, true);
        final PolygonResult areaEdgeResult1 = areaEdge.compute(false, true);

        //test reversed
        final PolygonResult polyEdgeResult2 = polyAreaEdge.compute(true, true);
        final PolygonResult areaEdgeResult2 = areaEdge.compute(true, true);

        //test remaining earth surface
        final PolygonResult polyEdgeResult3 = polyAreaEdge.compute(true, false);
        final PolygonResult areaEdgeResult3 = areaEdge.compute(true, false);

        final PolygonResult polyEdgeResult4 = polyAreaEdge.compute(false, false);
        final PolygonResult areaEdgeResult4 = areaEdge.compute(false, false);

        assertEquals(polyEdgeResult1.getNum(), testPolyResult1.getNum());
        assertEquals(polyEdgeResult1.getArea(), testPolyResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyEdgeResult1.getPerimeter(), testPolyResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyEdgeResult2.getNum(), testPolyResult2.getNum());
        assertEquals(polyEdgeResult2.getArea(), testPolyResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyEdgeResult2.getPerimeter(), testPolyResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyEdgeResult3.getNum(), testPolyResult3.getNum());
        assertEquals(polyEdgeResult3.getArea(), testPolyResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyEdgeResult3.getPerimeter(), testPolyResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(polyEdgeResult4.getNum(), testPolyResult4.getNum());
        assertEquals(polyEdgeResult4.getArea(), testPolyResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(polyEdgeResult4.getPerimeter(), testPolyResult4.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(areaEdgeResult1.getNum(), testAreaResult1.getNum());
        assertEquals(areaEdgeResult1.getArea(), testAreaResult1.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaEdgeResult1.getPerimeter(), testAreaResult1.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(areaEdgeResult2.getNum(), testAreaResult2.getNum());
        assertEquals(areaEdgeResult2.getArea(), testAreaResult2.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaEdgeResult2.getPerimeter(), testAreaResult2.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(areaEdgeResult3.getNum(), testAreaResult3.getNum());
        assertEquals(areaEdgeResult3.getArea(), testAreaResult3.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaEdgeResult3.getPerimeter(), testAreaResult3.getPerimeter(), ABSOLUTE_ERROR);

        assertEquals(areaEdgeResult4.getNum(), testAreaResult4.getNum());
        assertEquals(areaEdgeResult4.getArea(), testAreaResult4.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaEdgeResult4.getPerimeter(), testAreaResult4.getPerimeter(), ABSOLUTE_ERROR);
    }
}
