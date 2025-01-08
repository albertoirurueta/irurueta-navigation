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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class PolygonAreaTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    @Test
    void testConstructor() {
        //test with polyline
        var area = new PolygonArea(Geodesic.WGS84, true);

        //check
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, area.getMajorRadius(), 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, area.getFlattening(), 0.0);

        var p = area.getCurrentPoint();
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
    void testClear() {
        final var area = new PolygonArea(Geodesic.WGS84, true);

        area.clear();

        //check
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, area.getMajorRadius(), 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, area.getFlattening(), 0.0);

        final var p = area.getCurrentPoint();
        assertEquals(Double.NaN, p.getFirst(), 0.0);
        assertEquals(Double.NaN, p.getSecond(), 0.0);
    }

    @Test
    void testAddPointAndCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final var polyArea = new PolygonArea(Geodesic.WGS84, true);
        final var area = new PolygonArea(Geodesic.WGS84, false);

        final var lat1 = 41.382643;
        final var lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        final var lat2 = 41.382524;
        final var lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        final var lat3 = 41.382790;
        final var lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        final var lat4 = 41.382911;
        final var lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        var polyResult = polyArea.compute();
        var areaResult = area.compute();

        assertEquals(5, polyResult.getNum());
        assertEquals(121.34, polyResult.getPerimeter(), 1.0);
        assertEquals(Double.NaN, polyResult.getArea(), 0.0);

        assertEquals(5, areaResult.getNum());
        assertEquals(121.34, areaResult.getPerimeter(), 1.0);
        assertEquals(815.72, areaResult.getArea(), 1.0);
    }

    @Test
    void testAddEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final var lat1 = 41.382643;
        final var lon1 = 2.176700;

        final var lat2 = 41.382524;
        final var lon2 = 2.176861;

        final var lat3 = 41.382790;
        final var lon3 = 2.177210;

        final var lat4 = 41.382911;
        final var lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        var data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        var data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        var data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        var data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        final var polyArea = new PolygonArea(Geodesic.WGS84, true);
        final var area = new PolygonArea(Geodesic.WGS84, false);

        final var polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        final var areaEdge = new PolygonArea(Geodesic.WGS84, false);

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

        final var polyResult = polyArea.compute();
        final var areaResult = area.compute();

        final var polyEdgeResult = polyAreaEdge.compute();
        final var areaEdgeResult = areaEdge.compute();

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
    void testCompute() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final var area = new PolygonArea(Geodesic.WGS84, false);

        var lat = 41.382643;
        var lon = 2.176700;
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
        var result = area.compute(true, true);

        assertEquals(121.34, result.getPerimeter(), 1.0);
        assertEquals(-815.72, result.getArea(), 1.0);

        //compute remaining earth surface
        result = area.compute(true, false);

        assertNotNull(Geodesic.WGS84);
        final var earthArea = Geodesic.WGS84.getEllipsoidArea();

        assertEquals(earthArea - 815.72, result.getArea(), 1.0);
    }

    @Test
    void testTestPoint() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final var polyArea = new PolygonArea(Geodesic.WGS84, true);
        final var area = new PolygonArea(Geodesic.WGS84, false);

        final var lat1 = 41.382643;
        final var lon1 = 2.176700;
        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        final var lat2 = 41.382524;
        final var lon2 = 2.176861;
        polyArea.addPoint(lat2, lon2);
        area.addPoint(lat2, lon2);

        final var lat3 = 41.382790;
        final var lon3 = 2.177210;
        polyArea.addPoint(lat3, lon3);
        area.addPoint(lat3, lon3);

        final var lat4 = 41.382911;
        final var lon4 = 2.177009;
        polyArea.addPoint(lat4, lon4);
        area.addPoint(lat4, lon4);

        //test
        final var testPolyResult1 = polyArea.testPoint(lat1, lon1, false, true);
        final var testAreaResult1 = area.testPoint(lat1, lon1, false, true);

        //test reversed
        final var testPolyResult2 = polyArea.testPoint(lat1, lon1, true, true);
        final var testAreaResult2 = area.testPoint(lat1, lon1, true, true);

        //test remaining earth surface
        final var testPolyResult3 = polyArea.testPoint(lat1, lon1, true, false);
        final var testAreaResult3 = area.testPoint(lat1, lon1, true, false);

        final var testPolyResult4 = polyArea.testPoint(lat1, lon1, false, false);
        final var testAreaResult4 = area.testPoint(lat1, lon1, false, false);

        polyArea.addPoint(lat1, lon1);
        area.addPoint(lat1, lon1);

        //test
        final var polyResult1 = polyArea.compute(false, true);
        final var areaResult1 = area.compute(false, true);

        //test reversed
        final var polyResult2 = polyArea.compute(true, true);
        final var areaResult2 = area.compute(true, true);

        //test remaining earth surface
        final var polyResult3 = polyArea.compute(true, false);
        final var areaResult3 = area.compute(true, false);

        final var polyResult4 = polyArea.compute(false, false);
        final var areaResult4 = area.compute(false, false);

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
    void testTestEdge() {
        //define polygon around Plaça Sant Jaume, Barcelona using the following coordinates:
        //41.382643,2.176700
        //41.382524,2.176861
        //41.382790,2.177210
        //41.382911,2.177009

        final var lat1 = 41.382643;
        final var lon1 = 2.176700;

        final var lat2 = 41.382524;
        final var lon2 = 2.176861;

        final var lat3 = 41.382790;
        final var lon3 = 2.177210;

        final var lat4 = 41.382911;
        final var lon4 = 2.177009;

        assertNotNull(Geodesic.WGS84);
        final var data1 = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2);
        final var data2 = Geodesic.WGS84.inverse(lat2, lon2, lat3, lon3);
        final var data3 = Geodesic.WGS84.inverse(lat3, lon3, lat4, lon4);
        final var data4 = Geodesic.WGS84.inverse(lat4, lon4, lat1, lon1);

        final var polyAreaEdge = new PolygonArea(Geodesic.WGS84, true);
        final var areaEdge = new PolygonArea(Geodesic.WGS84, false);

        polyAreaEdge.addPoint(lat1, lon1);
        areaEdge.addPoint(lat1, lon1);

        polyAreaEdge.addEdge(data1.getAzi1(), data1.getS12());
        areaEdge.addEdge(data1.getAzi1(), data1.getS12());

        polyAreaEdge.addEdge(data2.getAzi1(), data2.getS12());
        areaEdge.addEdge(data2.getAzi1(), data2.getS12());

        polyAreaEdge.addEdge(data3.getAzi1(), data3.getS12());
        areaEdge.addEdge(data3.getAzi1(), data3.getS12());

        //test
        final var testPolyResult1 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, true);
        final var testAreaResult1 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, true);

        //test reversed
        final var testPolyResult2 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, true);
        final var testAreaResult2 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, true);

        //test remaining earth surface
        final var testPolyResult3 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, false);
        final var testAreaResult3 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), true, false);

        final var testPolyResult4 = polyAreaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, false);
        final var testAreaResult4 = areaEdge.testEdge(data4.getAzi1(), data4.getS12(), false, false);

        polyAreaEdge.addEdge(data4.getAzi1(), data4.getS12());
        areaEdge.addEdge(data4.getAzi1(), data4.getS12());

        //test
        final var polyEdgeResult1 = polyAreaEdge.compute(false, true);
        final var areaEdgeResult1 = areaEdge.compute(false, true);

        //test reversed
        final var polyEdgeResult2 = polyAreaEdge.compute(true, true);
        final var areaEdgeResult2 = areaEdge.compute(true, true);

        //test remaining earth surface
        final var polyEdgeResult3 = polyAreaEdge.compute(true, false);
        final var areaEdgeResult3 = areaEdge.compute(true, false);

        final var polyEdgeResult4 = polyAreaEdge.compute(false, false);
        final var areaEdgeResult4 = areaEdge.compute(false, false);

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
