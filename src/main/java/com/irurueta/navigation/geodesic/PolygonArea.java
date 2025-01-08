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

/**
 * Polygon areas.
 * This computes the area of a geodesic polygon using the method given Section 6 of
 * <ul>
 *     <li>
 *         C. F. F. Karney, <a href="https://doi.org/10.1007/s00190-012-0578-z">Algorithms for
 *         geodesics</a>,
 *         J. Geodesy <b>87</b>, 43&ndash;55 (2013)
 *     </li>
 * </ul>
 * This class lets you add vertices one at a time to the polygon. The area and perimeter are
 * accumulated at two times the standard floating point precision to guard against the loss of
 * accuracy with many-sided polygons.
 * At any point you can ask for the perimeter and area so far. There's an option to treat the
 * points as defining a polyline instead of a polygon; in that case, only the perimeter is
 * computed.
 * Example of use:
 * <pre>
 * {@code
 * // Compute the area of a geodesic polygon.
 *
 * // This program reads lines with lat, lon for each vertex of a polygon.
 * // At the end of input, the program prints the number of vertices,
 * // the perimeter of the polygon and its area (for the WGS84 ellipsoid).
 *
 * import java.util.*;
 * import com.irurueta.navigation.geodesic.*;
 *
 * public class Planimeter {
 *   public static void main(String[] args) {
 *     PolygonArea p = new PolygonArea(Geodesic.WGS84, false);
 *     try {
 *       Scanner in = new Scanner(System.in);
 *       while (true) {
 *         double lat = in.nextDouble(), lon = in.nextDouble();
 *         p.AddPoint(lat, lon);
 *       }
 *     }
 *     catch (Exception e) {}
 *     PolygonResult r = p.Compute();
 *     System.out.println(r.num + " " + r.perimeter + " " + r.area);
 *   }
 * }}</pre>
 */
@SuppressWarnings("DuplicatedCode")
public class PolygonArea {

    private final Geodesic earth;

    // full ellipsoid area
    private final double area0;

    // assume polyline (don't close and skip area)
    private final boolean polyline;

    private final int mask;
    private int num;
    private int crossings;

    private Accumulator areasum;
    private final Accumulator perimetersum;

    private double lat0;
    private double lon0;
    private double lat1;
    private double lon1;

    /**
     * Constructor for PolygonArea.
     *
     * @param earth    the Geodesic object to use for geodesic calculations.
     * @param polyline if true that treat the points as defining a polyline instead of a polygon.
     */
    public PolygonArea(final Geodesic earth, final boolean polyline) {
        this.earth = earth;
        area0 = this.earth.getEllipsoidArea();
        this.polyline = polyline;
        mask = GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE | GeodesicMask.DISTANCE |
                (this.polyline ? GeodesicMask.NONE : GeodesicMask.AREA | GeodesicMask.LONG_UNROLL);
        perimetersum = new Accumulator(0);
        if (!this.polyline) {
            areasum = new Accumulator(0);
        }
        clear();
    }

    /**
     * Clear PolygonArea, allowing a new polygon to be started.
     */
    public void clear() {
        num = 0;
        crossings = 0;
        perimetersum.set(0);
        if (!polyline) {
            areasum.set(0);
        }
        lat0 = lon0 = lat1 = lon1 = Double.NaN;
    }

    /**
     * Add a point to the polygon or polyline.
     * <i>lat</i> should be in the range [&minus;90&deg;, 90&deg;].
     *
     * @param lat the latitude of the point (degrees).
     * @param lon the latitude of the point (degrees).
     */
    public void addPoint(final double lat, double lon) {
        lon = GeoMath.angNormalize(lon);
        if (num == 0) {
            lat0 = lat1 = lat;
            lon0 = lon1 = lon;
        } else {
            final var g = earth.inverse(lat1, lon1, lat, lon, mask);
            perimetersum.add(g.getS12());
            if (!polyline) {
                areasum.add(g.getAreaS12());
                crossings += transit(lon1, lon);
            }
            lat1 = lat;
            lon1 = lon;
        }
        ++num;
    }

    /**
     * Add an edge to the polygon or polyline.
     * This does nothing if no points have been added yet. Use PolygonArea.getCurrentPoint to
     * determine the position of the new vertex.
     *
     * @param azi azimuth at current point (degrees).
     * @param s   distance from current point to next point (meters).
     */
    public void addEdge(final double azi, final double s) {
        // do nothing if mNum is zero
        if (num > 0) {
            final var g = earth.direct(lat1, lon1, azi, s, mask);
            perimetersum.add(g.getS12());
            if (!polyline) {
                areasum.add(g.getAreaS12());
                crossings += transitDirect(lon1, g.getLon2());
            }
            lat1 = g.getLat2();
            lon1 = g.getLon2();
            ++num;
        }
    }

    /**
     * Return the results so far.
     * Counter-clockwise traversal counts as a positive area.
     *
     * @return PolygonResult(<i>num</i>, <i>perimeter</i>, <i>area</i>) where
     * <i>num</i> is the number of vertices, <i>perimeter</i> is the perimeter of
     * the polygon or the length of the polyline (meters), and <i>area</i> is the
     * area of the polygon (meters<sup>2</sup>) or Double.NaN of <i>polyline</i>
     * is true in the constructor.
     */
    public PolygonResult compute() {
        return compute(false, true);
    }

    /**
     * Return the results so far.
     * More points can be added to the polygon after this call.
     *
     * @param reverse if true then clockwise (instead of counter-clockwise) traversal counts as
     *                a positive area.
     * @param sign    if true then return a signed result for the area if the polygon is traversed
     *                in the "wrong" direction instead of returning the area for the rest of the
     *                earth.
     * @return PolygonResult(<i>num</i>, <i>perimeter</i>, <i>area</i>) where
     * <i>num</i> is the number of vertices, <i>perimeter</i> is the perimeter of the polygon
     * or the length of the polyline (meters), and <i>area</i> is the area of the polygon
     * (meters<sup>2</sup>) or Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult compute(final boolean reverse, final boolean sign) {
        if (num < 2) {
            return new PolygonResult(num, 0, polyline ? Double.NaN : 0);
        }
        if (polyline) {
            return new PolygonResult(num, perimetersum.getSum(), Double.NaN);
        }

        final var g = earth.inverse(lat1, lon1, lat0, lon0, mask);
        final var tempsum = new Accumulator(areasum);
        tempsum.add(g.getAreaS12());
        final var tcrossings = this.crossings + transit(lon1, lon0);
        if ((tcrossings & 1) != 0) {
            tempsum.add((tempsum.getSum() < 0 ? 1 : -1) * area0 / 2);
        }

        // area is with the clockwise sense. If !reverse convert to counter-clockwise convention
        if (!reverse) {
            tempsum.negate();
        }

        // if sign put area in (-rea0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum.getSum() > area0 / 2) {
                tempsum.add(-area0);
            } else if (tempsum.getSum() <= -area0 / 2) {
                tempsum.add(+area0);
            }
        } else {
            if (tempsum.getSum() >= area0) {
                tempsum.add(-area0);
            } else if (tempsum.getSum() < 0) {
                tempsum.add(+area0);
            }
        }
        return new PolygonResult(num, perimetersum.sum(g.getS12()), 0 + tempsum.getSum());
    }

    /**
     * Return the results assuming a tentative final test point is added;
     * however, the data for the test point is not saved. This lets you report a running result
     * for the perimeter and area as the user moves the mouse cursor. Ordinary floating point
     * arithmetic is used to accumulate the data for the test point; thus the area and perimeter
     * returned are less accurate than if addPoint and compute are used.
     * <i>lat</i> should be in the range [&minus;90&deg;, 90&deg;].
     *
     * @param lat     the latitude of the test point (degrees).
     * @param lon     the longitude of the test point (degrees).
     * @param reverse if true then clockwise (instead of counter-clockwise) traversal counts as
     *                a positive area.
     * @param sign    if true then return a signed result for the area if the polygon is traversed
     *                in the "wrong" direction instead of returning the area for the rest of the
     *                earth.
     * @return PolygonResult(<i>num</i>, <i>perimeter</i>, <i>area</i>) where <i>num</i> is
     * the number of vertices, <i>perimeter</i> is the perimeter of the polygon or the length
     * of the polyline (meters), and <i>area</i> is the area of the polygon (meters<sup>2</sup>)
     * or Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult testPoint(final double lat, final double lon, final boolean reverse, final boolean sign) {
        if (num == 0) {
            return new PolygonResult(1, 0, polyline ? Double.NaN : 0);
        }

        var perimeter = perimetersum.getSum();
        var tempsum = polyline ? 0 : areasum.getSum();
        var tcrossings = this.crossings;
        final var tnum = this.num + 1;
        for (var i = 0; i < (polyline ? 1 : 2); ++i) {
            final var g = earth.inverse(i == 0 ? lat1 : lat, i == 0 ? lon1 : lon, i != 0 ? lat0 : lat,
                    i != 0 ? lon0 : lon, mask);
            perimeter += g.getS12();
            if (!polyline) {
                tempsum += g.getAreaS12();
                tcrossings += transit(i == 0 ? lon1 : lon, i != 0 ? lon0 : lon);
            }
        }

        if (polyline) {
            return new PolygonResult(tnum, perimeter, Double.NaN);
        }

        if ((tcrossings & 1) != 0) {
            tempsum += (tempsum < 0 ? 1 : -1) * area0 / 2;
        }

        // area is with the clockwise sense. If !reverse convert to counter-clockwise convention
        if (!reverse) {
            tempsum *= -1;
        }

        // if sign put area in (-area0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum > area0 / 2) {
                tempsum -= area0;
            } else if (tempsum <= -area0 / 2) {
                tempsum += area0;
            }
        } else {
            if (tempsum >= area0) {
                tempsum -= area0;
            } else if (tempsum < 0) {
                tempsum += area0;
            }
        }
        return new PolygonResult(tnum, perimeter, 0 + tempsum);
    }

    /**
     * Return the results assuming a tentative final test point is added via an azimuth and distance;
     * however, the data for the test point is not saved.
     * This lets you report a running result for the perimeter and area as the user moves the mouse
     * cursor. Ordinary floating point arithmetic is used to accumulate the data for the test point;
     * thus the area and perimeter returned are less accurate than if addPoint and compute are used.
     *
     * @param azi     azimuth at current point (degrees).
     * @param s       distance from current point to final test point (meters).
     * @param reverse if true then clockwise (instead of counter-clockwise) traversal counts as a
     *                positive area.
     * @param sign    if true then return a signed result for the area if the polygon is traversed in
     *                the "wrong" direction instead of returning the area for the rest of the earth.
     * @return PolygonResult(<i>num</i>, <i>perimeter</i>, <i>area</i>) where <i>num</i> is the
     * number of vertices, <i>perimeter</i> is the perimeter of the polygon or the length of the
     * polyline (meters), and <i>area</i> is the area of the polygon (meters<sup>2</sup>) or
     * Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult testEdge(
            final double azi, final double s, final boolean reverse, final boolean sign) {
        // we don't have a starting point!
        if (num == 0) {
            return new PolygonResult(0, Double.NaN, Double.NaN);
        }

        final var tnum = this.num + 1;
        var perimeter = perimetersum.getSum() + s;
        if (polyline) {
            return new PolygonResult(tnum, perimeter, Double.NaN);
        }

        var tempsum = areasum.getSum();
        var tcrossings = this.crossings;

        var g = earth.direct(lat1, lon1, azi, false, s, mask);
        tempsum += g.getAreaS12();
        tcrossings += transitDirect(lon1, g.getLon2());
        g = earth.inverse(g.getLat2(), g.getLon2(), lat0, lon0, mask);
        perimeter += g.getS12();
        tempsum += g.getAreaS12();
        tcrossings += transit(g.getLon2(), lon0);

        if ((tcrossings & 1) != 0) {
            tempsum += (tempsum < 0 ? 1 : -1) * area0 / 2;
        }

        // area is with the clockwise sense. If !reverse convert to counter-clockwise convention.
        if (!reverse) {
            tempsum *= -1;
        }

        // if sign put area in (-area0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum > area0 / 2) {
                tempsum -= area0;
            } else if (tempsum <= -area0 / 2) {
                tempsum += area0;
            }
        } else {
            if (tempsum >= area0) {
                tempsum -= area0;
            } else if (tempsum < 0) {
                tempsum += area0;
            }
        }

        return new PolygonResult(tnum, perimeter, 0 + tempsum);
    }

    /**
     * Gets the equatorial radius of the ellipsoid (meters).
     *
     * @return <i>a</i> the equatorial radius of the ellipsoid (meters). This is the value inherited
     * from the Geodesic object used in the constructor.
     */
    public double getMajorRadius() {
        return earth.getMajorRadius();
    }

    /**
     * Gets the flattening of the ellipsoid.
     *
     * @return <i>f</i> the flattening of the ellipsoid. This is the value inherited from the Geodesic
     * object used in the constructor.
     */
    public double getFlattening() {
        return earth.getFlattening();
    }

    /**
     * Report the previous vertex added to the polygon or polyline.
     * If no points have been added, then Double.NaN is returned. Otherwise, <i>lon</i> will be
     * in the range [&minus;180&deg;, 180&deg;].
     *
     * @return Pair(<i>lat</i>, <i>lon</i>), the current latitude and longitude.
     */
    public Pair getCurrentPoint() {
        return new Pair(lat1, lon1);
    }

    private static int transit(double lon1, double lon2) {
        // return 1 or -1 if crossing prime meridian in east or west direction.
        // Otherwise, return zero.
        // Compute lon12 the same way as Geodesic.inverse.
        lon1 = GeoMath.angNormalize(lon1);
        lon2 = GeoMath.angNormalize(lon2);

        final var lon12 = GeoMath.angDiff(lon1, lon2).getFirst();
        if (lon1 <= 0 && lon2 > 0 && lon12 > 0) {
            return 1;
        } else {
            return lon2 <= 0 && lon1 > 0 && lon12 < 0 ? -1 : 0;
        }
    }

    // an alternate version of transit to deal with longitudes in the direct problem.
    private static int transitDirect(double lon1, double lon2) {
        // we want to compute exactly
        // int(floor(lon2 / 360)) - int(floor(lon1 / 360))
        lon1 = lon1 % 720.0;
        lon2 = lon2 % 720.0;
        return (((lon2 >= 0 && lon2 < 360) || lon2 < -360 ? 0 : 1) -
                ((lon1 >= 0 && lon1 < 360) || lon1 < -360 ? 0 : 1));
    }
}
