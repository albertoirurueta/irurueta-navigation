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
 * accumulated at two times the standard floating point precision to guard agains the loss of
 * accuracy with many-sided polygons.
 * At any point you can ask for the permiter and area so far. There's an option to treat the
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
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class PolygonArea {

    private final Geodesic mEarth;

    //full ellipsoid area
    private final double mArea0;

    //assume polyline (don't close and skip area)
    private final boolean mPolyline;

    private final int mMask;
    private int mNum;
    private int mCrossings;

    private Accumulator mAreasum;
    private final Accumulator mPerimetersum;

    private double mLat0;
    private double mLon0;
    private double mLat1;
    private double mLon1;

    /**
     * Constructor for PolygonArea.
     *
     * @param earth    the Geodesic object to use for geodesic calculations.
     * @param polyline if true that treat the points as defining a polyline instead of a polygon.
     */
    @SuppressWarnings("WeakerAccess")
    public PolygonArea(final Geodesic earth, final boolean polyline) {
        mEarth = earth;
        mArea0 = mEarth.getEllipsoidArea();
        mPolyline = polyline;
        mMask = GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE |
                GeodesicMask.DISTANCE |
                (mPolyline ? GeodesicMask.NONE :
                        GeodesicMask.AREA | GeodesicMask.LONG_UNROLL);
        mPerimetersum = new Accumulator(0);
        if (!mPolyline) {
            mAreasum = new Accumulator(0);
        }
        clear();
    }

    /**
     * Clear PolygonArea, allowing a new polygon to be started.
     */
    public void clear() {
        mNum = 0;
        mCrossings = 0;
        mPerimetersum.set(0);
        if (!mPolyline) {
            mAreasum.set(0);
        }
        mLat0 = mLon0 = mLat1 = mLon1 = Double.NaN;
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
        if (mNum == 0) {
            mLat0 = mLat1 = lat;
            mLon0 = mLon1 = lon;
        } else {
            final GeodesicData g = mEarth.inverse(mLat1, mLon1, lat, lon, mMask);
            mPerimetersum.add(g.getS12());
            if (!mPolyline) {
                mAreasum.add(g.getAreaS12());
                mCrossings += transit(mLon1, lon);
            }
            mLat1 = lat;
            mLon1 = lon;
        }
        ++mNum;
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
        //do nothing if mNum is zero
        if (mNum > 0) {
            final GeodesicData g = mEarth.direct(mLat1, mLon1, azi, s, mMask);
            mPerimetersum.add(g.getS12());
            if (!mPolyline) {
                mAreasum.add(g.getAreaS12());
                mCrossings += transitDirect(mLon1, g.getLon2());
            }
            mLat1 = g.getLat2();
            mLon1 = g.getLon2();
            ++mNum;
        }
    }

    /**
     * Return the results so far.
     * Counter-clockwise traversal counts as a positive area.
     *
     * @return PolygonResult(< i > num < / i >, < i > perimeter < / i >, < i > area < / i >) where
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
     * @return PolygonResult(< i > num < / i >, < i > perimeter < / i >, < i > area < / i >) where
     * <i>num</i> is the number of vertices, <i>perimeter</i> is the perimeter of the polygon
     * or the length of the polyline (meters), and <i>area</i> is the area of the polygon
     * (meters<sup>2</sup>) or Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult compute(final boolean reverse, final boolean sign) {
        if (mNum < 2) {
            return new PolygonResult(mNum, 0, mPolyline ? Double.NaN : 0);
        }
        if (mPolyline) {
            return new PolygonResult(mNum, mPerimetersum.getSum(), Double.NaN);
        }

        final GeodesicData g = mEarth.inverse(mLat1, mLon1, mLat0, mLon0, mMask);
        final Accumulator tempsum = new Accumulator(mAreasum);
        tempsum.add(g.getAreaS12());
        final int crossings = mCrossings + transit(mLon1, mLon0);
        if ((crossings & 1) != 0) {
            tempsum.add((tempsum.getSum() < 0 ? 1 : -1) * mArea0 / 2);
        }

        //area is with the clockwise sense. If !reverse convert to counter-clockwise convention
        if (!reverse) {
            tempsum.negate();
        }

        //if sign put area in (-rea0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum.getSum() > mArea0 / 2) {
                tempsum.add(-mArea0);
            } else if (tempsum.getSum() <= -mArea0 / 2) {
                tempsum.add(+mArea0);
            }
        } else {
            if (tempsum.getSum() >= mArea0) {
                tempsum.add(-mArea0);
            } else if (tempsum.getSum() < 0) {
                tempsum.add(+mArea0);
            }
        }
        return new PolygonResult(mNum, mPerimetersum.sum(g.getS12()), 0 + tempsum.getSum());
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
     * @return PolygonResult(< i > num < / i >, < i > perimeter < / i >, < i > area < / i >) where <i>num</i> is
     * the number of vertices, <i>perimeter</i> is the perimeter of the polygon or the length
     * of the polyline (meters), and <i>area</i> is the area of the polygon (meters<sup>2</sup>)
     * or Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult testPoint(
            final double lat, final double lon, final boolean reverse, final boolean sign) {
        if (mNum == 0) {
            return new PolygonResult(1, 0, mPolyline ? Double.NaN : 0);
        }

        double perimeter = mPerimetersum.getSum();
        double tempsum = mPolyline ? 0 : mAreasum.getSum();
        int crossings = mCrossings;
        final int num = mNum + 1;
        for (int i = 0; i < (mPolyline ? 1 : 2); ++i) {
            final GeodesicData g = mEarth.inverse(i == 0 ? mLat1 : lat,
                    i == 0 ? mLon1 : lon,
                    i != 0 ? mLat0 : lat,
                    i != 0 ? mLon0 : lon, mMask);
            perimeter += g.getS12();
            if (!mPolyline) {
                tempsum += g.getAreaS12();
                crossings += transit(i == 0 ? mLon1 : lon, i != 0 ? mLon0 : lon);
            }
        }

        if (mPolyline) {
            return new PolygonResult(num, perimeter, Double.NaN);
        }

        if ((crossings & 1) != 0) {
            tempsum += (tempsum < 0 ? 1 : -1) * mArea0 / 2;
        }

        //area is with the clockwise sense. If !reverse convert to counter-clockwise convention
        if (!reverse) {
            tempsum *= -1;
        }

        //if sign put area in (-area0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum > mArea0 / 2) {
                tempsum -= mArea0;
            } else if (tempsum <= -mArea0 / 2) {
                tempsum += mArea0;
            }
        } else {
            if (tempsum >= mArea0) {
                tempsum -= mArea0;
            } else if (tempsum < 0) {
                tempsum += mArea0;
            }
        }
        return new PolygonResult(num, perimeter, 0 + tempsum);
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
     * @return PolygonResult(< i > num < / i >, < i > perimeter < / i >, < i > area < / i >) where <i>num</i> is the
     * number of vertices, <i>perimeter</i> is the perimeter of the polygon or the length of the
     * polyline (meters), and <i>area</i> is the area of the polygon (meters<sup>2</sup>) or
     * Double.NaN of <i>polyline</i> is true in the constructor.
     */
    public PolygonResult testEdge(
            final double azi, final double s, final boolean reverse, final boolean sign) {
        //we don't have a starting point!
        if (mNum == 0) {
            return new PolygonResult(0, Double.NaN, Double.NaN);
        }

        final int num = mNum + 1;
        double perimeter = mPerimetersum.getSum() + s;
        if (mPolyline) {
            return new PolygonResult(num, perimeter, Double.NaN);
        }

        double tempsum = mAreasum.getSum();
        int crossings = mCrossings;

        GeodesicData g = mEarth.direct(mLat1, mLon1, azi, false, s, mMask);
        tempsum += g.getAreaS12();
        crossings += transitDirect(mLon1, g.getLon2());
        g = mEarth.inverse(g.getLat2(), g.getLon2(), mLat0, mLon0, mMask);
        perimeter += g.getS12();
        tempsum += g.getAreaS12();
        crossings += transit(g.getLon2(), mLon0);

        if ((crossings & 1) != 0) {
            tempsum += (tempsum < 0 ? 1 : -1) * mArea0 / 2;
        }

        //area is with the clockwise sense. If !reverse convert to counter-clockwise convention.
        if (!reverse) {
            tempsum *= -1;
        }

        //if sign put area in (-area0/2, area0/2], else put area in [0, area0)
        if (sign) {
            if (tempsum > mArea0 / 2) {
                tempsum -= mArea0;
            } else if (tempsum <= -mArea0 / 2) {
                tempsum += mArea0;
            }
        } else {
            if (tempsum >= mArea0) {
                tempsum -= mArea0;
            } else if (tempsum < 0) {
                tempsum += mArea0;
            }
        }

        return new PolygonResult(num, perimeter, 0 + tempsum);
    }

    /**
     * Gets the equatorial radius of the ellipsoid (meters).
     *
     * @return <i>a</i> the equatorial radius of the ellipsoid (meters). This is the value inherited
     * from the Geodesic object used in the constructor.
     */
    public double getMajorRadius() {
        return mEarth.getMajorRadius();
    }

    /**
     * Gets the flattening of the ellipsoid.
     *
     * @return <i>f</i> the flattening of the ellipsoid. This is the value inherited from the Geodesic
     * object used in the constructor.
     */
    public double getFlattening() {
        return mEarth.getFlattening();
    }

    /**
     * Report the previous vertex added to the polygon or polyline.
     * If no points have been added, then Double.NaN is returned. Otherwise, <i>lon</i> will be
     * in the range [&minus;180&deg;, 180&deg;].
     *
     * @return Pair(< i > lat < / i >, < i > lon < / i >), the current latitude and longitude.
     */
    public Pair getCurrentPoint() {
        return new Pair(mLat1, mLon1);
    }

    private static int transit(double lon1, double lon2) {
        //return 1 or -1 if crossing prime meridian in east or west direction.
        //Otherwise return zero.
        //Compute lon12 the same way as Geodesic.inverse.
        lon1 = GeoMath.angNormalize(lon1);
        lon2 = GeoMath.angNormalize(lon2);

        final double lon12 = GeoMath.angDiff(lon1, lon2).getFirst();
        if (lon1 <= 0 && lon2 > 0 && lon12 > 0) {
            return 1;
        } else {
            return lon2 <= 0 && lon1 > 0 && lon12 < 0 ? -1 : 0;
        }
    }

    //an alternate version of transit to deal with longitudes in the direct problem.
    private static int transitDirect(double lon1, double lon2) {
        //we want to compute exactly
        //int(floor(lon2 / 360)) - int(floor(lon1 / 360))
        lon1 = lon1 % 720.0;
        lon2 = lon2 % 720.0;
        return (((lon2 >= 0 && lon2 < 360) || lon2 < -360 ? 0 : 1) -
                ((lon1 >= 0 && lon1 < 360) || lon1 < -360 ? 0 : 1));
    }
}
