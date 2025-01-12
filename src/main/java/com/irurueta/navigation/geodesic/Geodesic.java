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
 * Geodesic calculations.
 * The shortest path between two points on an ellipsoid at (<i>lat1</i>, <i>lon1</i>) and
 * (<i>lat2</i>, <i>lon2</i>) is called the geodesic. Its length is <i>s12</i> and the geodesic
 * from point 1 to point 2 has azimuths <i>azi1</i> and <i>azi2</i> at the two end points. (The
 * azimuth is the heading measured clockwise from north. <i>azi2</i> is the "forward" azimuth, i.e.,
 * the heading that takes you beyond point 2 not back to point 1.).
 * Given <i>lat1</i>, <i>lon1</i>, <i>azi1</i>, and <i>s12</i>, we can determine <i>lat2</i>,
 * <i>lon2</i>, and <i>azi2</i>. This is the <i>direct</i> geodesic problem and its solution is given
 * by th function {@link #direct}. (If <i>s12</i> is sufficiently large that the geodesic wraps
 * more than halfway around the earth, there will be another geodesic between the points with
 * a smaller <i>s12</i>.)
 * Given <i>lat1</i>, <i>lon1</i>, <i>lat2</i>, and <i>lon2</i>, we can determine <i>azi1</i>,
 * <i>azi2</i>, and <i>s12</i>. This is the <i>inverse</i> geodesic problem, whose solution is
 * given by {@link #inverse}. Usually, the solution to the inverse problem is unique. In cases where
 * there are multiple solutions (all with the same <i>s12</i>, of course), all the solutions can be
 * easily generated once a particular solution is provided.
 * The standard way of specifying the direct problem is to specify the distance <i>s12</i> to the
 * second point. However, it is sometimes useful instead to specify the arc length <i>a12</i> (in
 * degrees) on the auxiliary sphere. This is a mathematical construct used in solving the geodesic
 * problems. The solution of the direct problem in this form is provided by {@link #arcDirect}. An
 * arc length in excess of 180&deg; indicates that the geodesic is not the shortest path. In addition,
 * the arc length between an equatorial crossing and the next extremum of latitude for a geodesic is
 * 90&deg;.
 * This class can also calculate several other quantities related to geodesics. These are:
 * <ul>
 *     <li>
 *         <i>reduced length</i>. If we fix the first point and increase <i>azi1</i> by <i>dazi1</i>
 *         (radians), the second point is displaced <i>m12</i> <i>dazi1</i> in the direction
 *         <i>azi2</i> + 90&deg;. The quantity <i>m12</i> is called the "reduced length" and is
 *         symmetric under interchange of the two points. On a curved surface the reduced length
 *         obeys a symmetry relation, <i>m12</i> + <i>m21</i> = 0. On a flat surface, we have
 *         <i>m12</i> = <i>s12</i>. The ratio <i>s12</i>/<i>m12</i> gives the azimuthal scale for
 *         an azimuthal equidistant projection.
 *     </li>
 *     <li>
 *         <i>geodesic scale</i>. Consider a reference geodesic and a second geodesic parallel to this
 *         one at point 1 and separated by a small distance <i>dt</i>. The separation of the two
 *         geodesics at point 2 is <i>M12</i><i>dt</i> where <i>M12</i> is called the "geodesic scale".
 *         <i>M21</i> is defined similarly (with the geodesics being parallel at point 2). On a flat
 *         surface, we have <i>M12</i> = <i>M21</i> = 1. The quantity 1/<i>M12</i> gives the scale
 *         of the Cassini-Soldner projection.
 *     </li>
 *     <li>
 *         <i>area</i>. The area between the geodesic from point 1 to point 2 and the equation is
 *         represented by <i>S12</i>; it is the area, measured counter-clockwise, of the geodesic
 *         quadrilateral with corners (<i>lat1</i>, <i>lon1</i>), (0, <i>lon1</i>), (0, <i>lon2</i>), and
 *         (<i>lat2</i>, <i>lon2</i>). It can be used to compute the area of any single geodesic
 *         polygon.
 *     </li>
 * </ul>
 * The quantities <i>m12</i>, <i>M12</i>, <i>M21</i> which all specify the behavior of nearby geodesics
 * obey addition rules. If points 1, 2, and 3 all lie on a single geodesic, then the following rules hold:
 * <ul>
 *     <li>
 *         <i>s13</i> = <i>s12</i> + <i>s23</i>
 *     </li>
 *     <li>
 *         <i>a13</i> = <i>a12</i> + <i>a23</i>
 *     </li>
 *     <li>
 *         <i>S13</i> = <i>S12</i> + <i>S23</i>
 *     </li>
 *     <li>
 *         <i>m13</i> = <i>m12</i> <i>M23</i> + <i>m23</i> <i>M21</i>
 *     </li>
 *     <li>
 *         <i>M13</i> = <i>M12</i> <i>M23</i> &minus; (1 &minus; <i>M12</i> <i>M21</i>) <i>m23</i> / <i>m12</i>
 *     </li>
 *     <li>
 *         <i>M31</i> = <i>M32</i> <i>M21</i> &minus; (1 &minus; <i>M23</i> <i>M32</i>) <i>m12</i> / <i>m23</i>
 *     </li>
 * </ul>
 * The results of the geodesic calculations are bundled up into a {@link GeodesicData} object which includes the
 * input parameters and all the computed results, i.e. <i>lat1</i>, <i>lon1</i>, <i>azi1</i>, <i>lat2</i>,
 * <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>, <i>m12</i>, <i>M12</i>, <i>M21</i>, <i>S12</i>.
 * The functions {@link #direct(double, double, double, double, int)},
 * {@link #arcDirect(double, double, double, double, int)} and {@link #inverse(double, double, double, double, int)}
 * include an optional final argument <i>outmask</i> which allows you to specify which results should be computed and
 * returned. If you omit <i>outmask</i>, then the "standard" geodesic results are computed (latitudes,  longitudes,
 * azimuths, and distance). <i>outmask</i> is bitor'ed combination of {@link GeodesicMask} values. For example, if
 * you wish just to compute the distance between two points you would call, e.g.,
 * <pre>
 *     {@code
 *     GeodesicData g = Geodesic.WGS84.inverse(lat1, lon1, lat2, lon2, GeodesicMask.DISTANCE);
 *     }
 * </pre>
 * Additional functionality is provided by the {@link GeodesicLine} class, which allows a sequence of points along a
 * geodesic to be computed.
 * The shortest distance returned by the solution of the inverse problem is (obviously) uniquely defined. However, in
 * a few special cases there are multiple azimuths which yield the same shortest distance. Here is a catalog of those
 * cases:
 * <ul>
 *     <li>
 *         <i>lat1</i> = &minus;<i>lat2</i> (with neither point at a pole). If
 *         <i>azi1</i> = <i>azi2</i>, the geodesic is unique. Otherwise there are two geodesics and the second one is
 *         obtained by setting [<i>azi1</i>, <i>azi2</i>] &rarr; [<i>azi2</i>, <i>azi1</i>], [<i>M12</i>, <i>M21</i>]
 *         &rarr; [<i>M21</i>, <i>M12</i>], <i>S12</i> &rarr; &minus;<i>S12</i>.
 *         (This occurs when the longitude difference is near &plusmn;180&deg; for oblate ellipsoids.)
 *     </li>
 *     <li>
 *         <i>lon2</i> = <i>lon1</i> + 180&deg; (with neither point at pole). If <i>azi1</i> = 0&deg; or
 *         &plusmn;180&deg;, the geodesic is unique. Otherwise there are two geodesics and the second one is obtained
 *         by setting [<i>azi1</i>, <i>azi2</i>] &rarr; [&minus;<i>azi1</i>, &minus;<i>azi2</i>], <i>S12</i> &rarr;
 *         &minus; <i>S12</i>. (This occurs when <i>lat2</i> is near &minus;<i>lat1</i> for prolate ellipsoids.)
 *     </li>
 *     <li>
 *         Points 1 and 2 at opposite poles. There are infinitely many geodesics which can be generated by setting
 *         [<i>azi1</i>, <i>azi2</i>] &rarr; [<i>azi1</i>, <i>azi2</i>] + [<i>d</i>, &minus;<i>d</i>], for arbitrary
 *         <i>d</i>. (For spheres, this prescription applies when points 1 and 2 are antipodal.)
 *     </li>
 *     <li>
 *         <i>s12</i> = 0 (coincident points). There are infinitely many geodesics which can be generated by setting
 *         [<i>azi1</i>, <i>azi2</i>] &rarr; [<i>azi1</i>, <i>azi2</i>] + [<i>d</i>, <i>d</i>], for arbitrary <i>d</i>.
 *     </li>
 * </ul>
 * The calculations are accurate to better than 15nm (15 nanometers) for the WGS84 ellipsoid. See Sec. 9 of
 * <a href="https://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a> for details. The algorithms used by this class are
 * based on series expansions using the flattening <i>f</i> as a small parameter. These are only accurate for |<i>f</i>|
 * &lt; 0.02; however reasonably accurate results will be obtained for |<i>f</i>| &lt; 0.2. Here is a table with the
 * same equatorial radius as the WGS84 ellipsoid and different values of the flattening.
 * <pre>
 *     |f|      error
 *     0.01     25 nm
 *     0.02     30 nm
 *     0.05     10 um
 *     0.1      1.5 mm
 *     0.2      300 mm
 * </pre>
 * The algorithms are described in
 * <ul>
 *     <li>
 *         C.F.F Karney, <a href="https://doi.org/10.1007/s00190-012-0578-z">Algorithms for geodesics</a>,
 *         J. Geodesy <b>87</b>, 43&ndash;55 (2013)
 *     </li>
 * </ul>
 * Example of use:
 * <pre>
 *     {@code
 *     //Solve the direct geodesic problem.
 *
 *     //This program reads in lines with lat1, lon1, azi1, s12 and prints out lines with lat2, lon2, azi2
 *     //(for the WGS84 ellipsoid).
 *
 *     import java.util.*;
 *     import com.irurueta.navigation.geodesic.*;
 *
 *     public class Direct {
 *         public static void main(String[] args) {
 *             try {
 *                 Scanner in = new Scanner(System.in);
 *                 double lat1, lon1, azi1, s12;
 *                 while (true) {
 *                     lat1 = in.nextDouble();
 *                     lon1 = in.nextDouble();
 *                     azi1 = in.nextDouble();
 *                     s12 = in.nextDouble();
 *
 *                     GeodesicData g = Geodesic.WGS84.direct(lat1, lon1, azi1, s12);
 *                     System.out.println(g.lat2 + " " + g.lon2 + " " + g.azi2);
 *                 }
 *             } catch (Exception e) { }
 *         }
 *     }
 *     }
 * </pre>
 */
@SuppressWarnings("DuplicatedCode")
public class Geodesic {

    /**
     * A global instantiation of Geodesic with the parameters for the WGS84 ellipsoid.
     */
    public static final Geodesic WGS84 = safeInstance(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
            Constants.EARTH_FLATTENING_WGS84);

    /**
     * The order of the expansions used.
     */
    protected static final int GEODESIC_ORDER = 6;

    protected static final int NA1 = GEODESIC_ORDER;
    protected static final int NC1 = GEODESIC_ORDER;
    protected static final int NC1P = GEODESIC_ORDER;

    protected static final int NA2 = GEODESIC_ORDER;
    protected static final int NC2 = GEODESIC_ORDER;

    protected static final int NA3 = GEODESIC_ORDER;

    protected static final int NA3X = NA3;
    protected static final int NC3 = GEODESIC_ORDER;

    protected static final int NC3X = (NC3 * (NC3 - 1)) / 2;
    protected static final int NC4 = GEODESIC_ORDER;

    protected static final int NC4X = (NC4 * (NC4 + 1)) / 2;

    /**
     * Underflow guard. We require TINY * epsilon() &lt; 0 and TINY + epsilon() == epsilon()
     */
    protected static final double TINY = Math.sqrt(GeoMath.MIN);

    private static final int MAXIT1 = 20;
    private static final int MAXIT2 = MAXIT1 + GeoMath.DIGITS + 10;

    private static final double TOL0 = GeoMath.EPSILON;

    /**
     * Increase multiplier in defn of TOl1 from 100 to 200 to fix inverse case
     * 52.784459412564 0 -52.784459512563990912 179.634407464943777557
     */
    private static final double TOL1 = 200 * TOL0;

    private static final double TOL2 = Math.sqrt(TOL0);

    /**
     * Check on bisection interval.
     */
    private static final double TOLB = TOL0 * TOL2;

    private static final double XTHRESH = 1000 * TOL2;

    protected final double a;

    protected final double f;

    protected final double f1;

    protected final double e2;

    protected final double ep2;

    protected final double b;

    protected final double c2;

    private final double n;

    private final double etol2;

    private final double[] a3x;

    private final double[] c3x;

    private final double[] c4x;

    /**
     * Constructor for an ellipsoid with.
     *
     * @param a equatorial radius (meters).
     * @param f flattening of ellipsoid. Setting <i>f</i> = 0 gives a sphere. Negative <i>f</i> gives a prolate
     *          ellipsoid.
     * @throws GeodesicException if <i>a</i> or (1 &minus; <i>f</i>) <i>a</i> is not positive.
     */
    public Geodesic(final double a, final double f) throws GeodesicException {
        this.a = a;
        this.f = f;
        f1 = 1 - this.f;
        e2 = this.f * (2 - this.f);

        // e2 / (1 - e2)
        ep2 = e2 / GeoMath.sq(f1);
        n = this.f / (2 - this.f);
        b = this.a * f1;

        // authalic radius squared
        final var tmp = (e2 > 0 ? GeoMath.atanh(Math.sqrt(e2)) : Math.atan(Math.sqrt(-e2)));
        c2 = (GeoMath.sq(this.a) + GeoMath.sq(b) * (e2 == 0 ? 1 : tmp / Math.sqrt(Math.abs(e2)))) / 2;

        // The sig12 threshold for "really short". Using the auxiliary sphere solution with dnm computed at
        // (bet1 + bet2) / 2, the relative error in the azimuth consistency check is
        // sig12^2 * abs(f) * min(1, 1 - f/2) / 2.
        // (Error measured for 1/100 < b/a < 100 and abs(f) >= 1/1000. For a given f and sig12, the max error occurs for
        // lines near the pole. If the old rule for computing dnm = (dn1 + dn2)/2 is used, then the error increases by
        // a factor of 2.) Setting this equal to epsilon gives sig12 = etol2. Here 0.1 is a safety factor (error
        // decreased by 100) and max(0.001, abs(f)) stops etol2 getting too large in the nearly spherical case.
        etol2 = 0.1 * TOL2 / Math.sqrt(Math.max(0.001, Math.abs(this.f)) * Math.min(1.0, 1 - this.f / 2) / 2);

        if (!(GeoMath.isFinite(this.a) && this.a > 0)) {
            throw new GeodesicException("Equatorial radius is not positive");
        }
        if (!(GeoMath.isFinite(b) && b > 0)) {
            throw new GeodesicException("Polar semi-axis is not positive");
        }
        a3x = new double[NA3X];
        c3x = new double[NC3X];
        c4x = new double[NC4X];

        a3coeff();
        c3coeff();
        c4coeff();
    }

    /**
     * Solve the direct geodesic problem where the length of the geodesic is specified in terms of distance.
     * If either point is at a pole, the azimuth is defined by keeping the longitude fixed, writing
     * <i>lat</i> = &plusmn;(90&deg; &minus; &epsilon;), and taking the limit &epsilon; &rarr; 0+. An arc length greater
     * than 180&deg; signifies a geodesic which is not the shortest path. (For a prolate ellipsoid, an additional
     * condition is necessary for the shortest path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param s12  distance between point 1 and point 2 (meters); it can be negative.
     * @return a {@link GeodesicData} object with the following fields: <i>lat1</i>, <i>lon1</i>, <i>azi1</i>,
     * <i>lat2</i>, <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>. The values of <i>lon2</i> and <i>azi2</i>
     * returned are in the range [&minus;180&deg;, 180&deg;].
     */
    public GeodesicData direct(final double lat1, final double lon1, final double azi1, final double s12) {
        return direct(lat1, lon1, azi1, false, s12, GeodesicMask.STANDARD);
    }

    /**
     * Solve the direct geodesic problem where the length of the geodesic is specified in terms of distance with a
     * subset of the geodesic results returned.
     *
     * @param lat1    latitude of point 1 (degrees).
     * @param lon1    longitude of point 1 (degrees).
     * @param azi1    azimuth at point 1 (degrees).
     * @param s12     distance between point 1 and point 2 (meters); it can be negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results should be returned.
     * @return a {@link GeodesicData} object with the fields specified by <i>outmask</i> computed. <i>lat1</i>,
     * <i>lon1</i>, <i>azi1</i>, <i>s12</i>, and <i>a12</i> are always included in the returned result. The value of
     * <i>lon2</i> returned is in the range [&minus;180&deg;, 180&deg;], unless the <i>outmask</i> includes the
     * {@link GeodesicMask#LONG_UNROLL} flag.
     */
    public GeodesicData direct(
            final double lat1, final double lon1, final double azi1, final double s12, final int outmask) {
        return direct(lat1, lon1, azi1, false, s12, outmask);
    }

    /**
     * Solve the direct geodesic problem where the length of the geodesic is specified in terms of arc length.
     * If either point is at a pole, the azimuth is defined by keeping the longitude fixed, writing
     * <i>lat</i> = &plusmn;(90&deg; &minus; &epsilon;), and taking the limit &epsilon; &rarr; 0+. An arc length
     * greater than 180&deg; signifies a geodesic which is not the shortest path. (For a prolate ellipsoid, an additional
     * condition is necessary for the shortest path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param a12  arc length between point 1 and point 2 (degrees); it can be negative.
     * @return a {@link GeodesicData} object with the following fields: <i>lat1</i>, <i>lon1</i>, <i>azi1</i>,
     * <i>lat2</i>, <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>. The values of <i>lon2</i> and <i>azi2</i>
     * returned are in the range [&minus;180&deg;, 180&deg;].
     */
    public GeodesicData arcDirect(final double lat1, final double lon1, final double azi1, final double a12) {
        return direct(lat1, lon1, azi1, true, a12, GeodesicMask.STANDARD);
    }

    /**
     * Solve the direct geodesic problem where the length of the geodesic is specified in terms of arc length and with
     * a subset of the geodesic results returned.
     *
     * @param lat1    latitude of point 1 (degrees).
     * @param lon1    longitude of point 1 (degrees).
     * @param azi1    azimuth at point 1 (degrees).
     * @param a12     arc length between point 1 and point 2 (degrees); it can be negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results should be returned.
     * @return a {@link GeodesicData} object with the fields specified by <i>outmask</i> computed. <i>lat1</i>,
     * <i>lon1</i>, <i>azi1</i>, and <i>a12</i> are always included in the returned result. The value of <i>lon2</i>
     * returned is in the range [&minus;180&deg;, 180&deg;], unless the <i>outmask</i> includes the
     * {@link GeodesicMask#LONG_UNROLL} flag.
     */
    public GeodesicData arcDirect(
            final double lat1, final double lon1, final double azi1, final double a12, final int outmask) {
        return direct(lat1, lon1, azi1, true, a12, outmask);
    }

    /**
     * The general direct geodesic problem. {@link #direct} and {@link #arcDirect} are defined in terms of this
     * function.
     * The {@link GeodesicMask} values possible for <i>outmask</i> are
     * <ul>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LATITUDE} for the latitude <i>lat2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LONGITUDE} for the longitude <i>lon2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#AZIMUTH} for the azimuth <i>azi2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#DISTANCE} for the distance <i>s12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#REDUCED_LENGTH} for the reduced length <i>m12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#GEODESIC_SCALE} for the geodesic scales <i>M12</i> and <i>M21</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#AREA} for the area <i>S12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LONG_UNROLL}, if set then <i>lon1</i> is unchanged and <i>lon2</i>
     *         &minus; <i>lon1</i> indicates how many times and in what sense the geodesic encircles the ellipsoid.
     *         Otherwise <i>lon1</i> and <i>lon2</i> are both reduced to the range [&minus;180&deg;, 180&deg;].
     *     </li>
     * </ul>
     * The function value <i>a12</i> is always computed and returned and this equals <i>s12A12</i> is <i>arcmode</i> is
     * true. If <i>outmask</i> includes {@link GeodesicMask#DISTANCE} and <i>arcmode</i> is false, then <i>s12</i> =
     * <i>s12A12</i>. It is not necessary to include {@link GeodesicMask#DISTANCE_IN} in <i>outmask</i>; this is
     * automatically included if <i>arcmode</i> is false.
     *
     * @param lat1    latitude of point 1 (degrees).
     * @param lon1    longitude of point 1 (degrees).
     * @param azi1    azimuth at point 1 (degrees).
     * @param arcmode boolean flag determining the meaning of the <i>s12A12</i>.
     * @param s12A12  <i>arcmode</i> is false, this is the distance between point 1 and point 2 (meters); otherwise it
     *                is the arc length between point 1 and point 2 (degrees); it can be negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results should be returned.
     * @return a {@link GeodesicData} object with the fields specified by <i>outmask</i> computed.
     */
    public GeodesicData direct(
            final double lat1, final double lon1, final double azi1, final boolean arcmode, final double s12A12,
            int outmask) {
        // automatically supply DISTANCE_IN if necessary
        if (!arcmode) {
            outmask |= GeodesicMask.DISTANCE_IN;
        }
        return new GeodesicLine(this, lat1, lon1, azi1, outmask).position(arcmode, s12A12, outmask);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the direct geodesic problem specified in terms of
     * distance with all capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the direct geodesic
     * problem.
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param s12  distance between point 1 and point 2 (meters); it can be negative.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine directLine(
            final double lat1, final double lon1, final double azi1, final double s12) {
        return directLine(lat1, lon1, azi1, s12, GeodesicMask.ALL);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the direct geodesic problem specified in terms of
     * distance with a subset of the capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the direct geodesic
     * problem.
     *
     * @param lat1 latitude of point 1 (Degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param s12  distance between point 1 and point 2 (meters); it can be negative.
     * @param caps bitor'ed combination of {@link GeodesicMask} values specifying the capabilities the
     *             GeodesicLine object should possess, i.e., which quantities can be returned in calls
     *             to {@link GeodesicLine#position}.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine directLine(
            final double lat1, final double lon1, final double azi1, final double s12, final int caps) {
        return genDirectLine(lat1, lon1, azi1, false, s12, caps);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the direct geodesic problem specified in terms of
     * arc length with all capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the direct geodesic
     * problem.
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param a12  arc length between point 1 and point 2 (degrees); it can be negative.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine arcDirectLine(final double lat1, final double lon1, final double azi1, final double a12) {
        return arcDirectLine(lat1, lon1, azi1, a12, GeodesicMask.ALL);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the direct geodesic problem specified in terms of
     * arc length with a subset of the capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the direct geodesic problem.
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param a12  arc length between point 1 and point 2 (degrees); it can be negative.
     * @param caps bitor'ed combination of {@link GeodesicMask} values specifying the capabilities the
     *             GeodesicLine object should possess, i.e., which quantities can be returned in calls
     *             to {@link GeodesicLine#position}.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine arcDirectLine(
            final double lat1, final double lon1, final double azi1, final double a12, final int caps) {
        return genDirectLine(lat1, lon1, azi1, true, a12, caps);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the direct geodesic problem specified in terms of
     * either distance or arc length with a subset of the capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the direct geodesic
     * problem.
     *
     * @param lat1    latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1    longitude of point 1 (degrees).
     * @param azi1    azimuth at point 1 (degrees).
     * @param arcmode boolean flag determining the meaning of the <i>s12A12</i>.
     * @param s12A12  if <i>arcmode</i> is false, this is the distance between point 1 and point 2 (meters); otherwise
     *                it is the arc length between point 1 and point 2 (degrees); it can be negative.
     * @param caps    bitor'ed combination of {@link GeodesicMask} values specifying the capabilities the
     *                GeodesicLine object should possess, i.e., which quantities can be returned in calls
     *                to {@link GeodesicLine#position}.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine genDirectLine(
            final double lat1, final double lon1, double azi1, final boolean arcmode, final double s12A12, int caps) {
        azi1 = GeoMath.angNormalize(azi1);
        final double salp1;
        final double calp1;

        // Guard against underflow in salp0. Also -0 is converted to +0.
        final var p = GeoMath.sincosd(GeoMath.angRound(azi1));
        salp1 = p.getFirst();
        calp1 = p.getSecond();

        // Automatically supply DISTANCE_IN if necessary
        if (!arcmode) {
            caps |= GeodesicMask.DISTANCE_IN;
        }

        return new GeodesicLine(this, lat1, lon1, azi1, salp1, calp1, caps, arcmode, s12A12);
    }

    /**
     * Solve the inverse geodesic problem.
     * <i>lat1</i> and <i>lat2</i> should be in the range [&minus;90&deg;, 90&deg;]. The values of
     * <i>azi1</i> and <i>azi2</i> returned are in the range [&minus;180&deg;, 180&deg;].
     * If either point is at a pole, the azimuth is defined by keeping the longitude fixed, writing
     * <i>lat</i> = &plusmn;(90&deg; &minus; &epsilon;), taking the limit &epsilon; &rarr; 0+.
     * The solution to the inverse problem is found using Newton's method. If this fails to converge
     * (this is very unlikely in geodetic applications but does occur for very eccentric ellipsoids),
     * then the bisection method is used to refine the solution.
     *
     * @param lat1 latitude of point 1 (degrees).
     * @param lon1 longitude of point 1 (degrees).
     * @param lat2 latitude of point 2 (degrees).
     * @param lon2 longitude of point 2 (degrees).
     * @return a {@link GeodesicData} object with the following fields: <i>lat1</i>, <i>lon1</i>,
     * <i>azi1</i>, <i>lat2</i>, <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>.
     */
    public GeodesicData inverse(final double lat1, final double lon1, final double lat2, final double lon2) {
        return inverse(lat1, lon1, lat2, lon2, GeodesicMask.STANDARD);
    }

    /**
     * Solve the inverse geodesic problem with a subset of the geodesic results returned.
     * The {@link GeodesicMask} values possible for <i>outmask</i> are
     * <ul>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#DISTANCE} for the distance <i>s12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#AZIMUTH} for the latitude <i>azi2</i>.
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#REDUCED_LENGTH} for the reduced length
     *         <i>m12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#GEODESIC_SCALE} for the geodesic scales
     *         <i>M12</i> and <i>M21</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#AREA} for the area <i>S12</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#ALL} for all of the above.
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LONG_UNROLL}, if set then <i>lon1</i> is
     *         unchanged and <i>lon2</i> &minus; <i>lon1</i> indicates whether the geodesic is
     *         east going or west going. Otherwise <i>lon1</i> and <i>lon2</i> are both reduced to
     *         the range [&minus;180&deg;, 180&deg;].
     *     </li>
     * </ul>
     *
     * @param lat1    latitude of point 1 (degrees).
     * @param lon1    longitude of point 1 (degrees).
     * @param lat2    latitude of point 2 (degrees).
     * @param lon2    longitude of point 2 (degrees)
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which
     *                results should be returned.
     * @return a {@link GeodesicData} object with the fields specified by <i>outmask</i> computed.
     * <i>lat1</i>, <i>lon1</i>, <i>lat2</i>, <i>lon2</i>, and <i>a12</i> are always included in the
     * returned result.
     */
    public GeodesicData inverse(
            final double lat1, final double lon1, final double lat2, final double lon2, int outmask) {
        outmask &= GeodesicMask.OUT_MASK;
        final var result = inverseInt(lat1, lon1, lat2, lon2, outmask);
        final var r = result.g;

        if ((outmask & GeodesicMask.AZIMUTH) != 0) {
            r.setAzi1(GeoMath.atan2d(result.salp1, result.calp1));
            r.setAzi2(GeoMath.atan2d(result.salp2, result.calp2));
        }
        return r;
    }

    /**
     * Define a {@link GeodesicLine} in terms of the inverse geodesic problem with all capabilities
     * included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the inverse
     * geodesic problem.
     * <i>lat2</i> and <i>lat2</i> should be in the range [&minus;90&deg;, 90&deg;].
     *
     * @param lat1 latitude of point 1 (degrees).
     * @param lon1 longitude of point 1 (degrees).
     * @param lat2 latitude of point 2 (degrees).
     * @param lon2 longitude of point 2 (degrees).
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine inverseLine(final double lat1, final double lon1, final double lat2, final double lon2) {
        return inverseLine(lat1, lon1, lat2, lon2, GeodesicMask.ALL);
    }

    /**
     * Define a {@link GeodesicLine} in terms of the inverse geodesic problem with a subet of the
     * capabilities included.
     * This function sets point 3 of the GeodesicLine to correspond to point 2 of the inverse
     * geodesic problem.
     * <i>lat1</i> and <i>lat2</i> should be in the range [&minus;90&deg;, 90&deg;].
     *
     * @param lat1 latitude of point 1 (degrees).
     * @param lon1 longitude of point 1 (degrees).
     * @param lat2 latitude of point 2 (degrees).
     * @param lon2 longitude of point 2 (degrees).
     * @param caps bitor'ed combination of {@link GeodesicMask} values specifying the capabilities
     *             the GeodesicLine object should possess, i.e., which quantities can be returned
     *             in calls to {@link GeodesicLine#position}.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine inverseLine(
            final double lat1, final double lon1, final double lat2, final double lon2, int caps) {
        final var result = inverseInt(lat1, lon1, lat2, lon2, 0);
        final var salp1 = result.salp1;
        final var calp1 = result.calp1;
        final var azi1 = GeoMath.atan2d(salp1, calp1);
        final var a12 = result.g.getA12();
        // ensure that a12 can be converted to a distance
        if ((caps & (GeodesicMask.OUT_MASK & GeodesicMask.DISTANCE_IN)) != 0) {
            caps |= GeodesicMask.DISTANCE;
        }
        return new GeodesicLine(this, lat1, lon1, azi1, salp1, calp1, caps, true, a12);
    }

    /**
     * Set up to compute several points on a single geodesic with all capabilities included.
     * If the point is at a pole, the azimuth is defined by keeping the <i>lon1</i> fixed, writing
     * <i>lat1</i> = &plusmn; (90 &minus; &epsilon;), taking the limit &epsilon; &rarr; 0+.
     *
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range
     *             [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @return a {@link GeodesicLine} object. The full set of capabilities is included.
     */
    public GeodesicLine line(final double lat1, final double lon1, final double azi1) {
        return line(lat1, lon1, azi1, GeodesicMask.ALL);
    }

    /**
     * Set up to compute several points on a single geodesic with a subset of the capabilities
     * included.
     * The {@link GeodesicMask} values are:
     * <ul>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#LATITUDE} for the latitude <i>lat2</i>; this is
     *         added automatically;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#LONGITUDE} for the longitude <i>lon2</i>;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#AZIMUTH} for the azimuth <i>azi2</i>; this is
     *         added automatically;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#DISTANCE} for the distance <i>s12</i>;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#REDUCED_LENGTH} for the reduced length <i>m12</i>;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#GEODESIC_SCALE} for the geodesic scales <i>M12</i>
     *         and <i>M21</i>;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#AREA} for the area <i>S12</i>;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#DISTANCE_IN} permits the length of the geodesic to
     *         be given in terms of <i>s12</i>; without this capability the length can only be specified
     *         in terms of arc length;
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#ALL} for all of the above.
     *     </li>
     * </ul>
     * If the point is at a pole, the azimuth is defined by keeping <i>lon1</i> fixed, writing
     * <i>lat1</i> = &plusmn; (90 &minus; &epsilon;), and taking the limit &epsilon; &rarr; 0+.
     *
     * @param lat1 latitude of point 1 (degrees).
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param caps bitor'ed combination of {@link GeodesicMask} values specifying which
     *             quantities can be returned in calls to {@link GeodesicLine#position}.
     * @return a {@link GeodesicLine} object.
     */
    public GeodesicLine line(final double lat1, final double lon1, final double azi1, final int caps) {
        return new GeodesicLine(this, lat1, lon1, azi1, caps);
    }

    /**
     * Gets the equatorial radius of the ellipsoid (meters). This is the value used in the
     * constructor.
     *
     * @return <i>a</i> the equatorial radius of the ellipsoid (meters).
     */
    public double getMajorRadius() {
        return a;
    }

    /**
     * Gets the flattening of the ellipsoid. This is the value used in the constructor.
     *
     * @return <i>f</i> the flattening of the ellipsoid.
     */
    public double getFlattening() {
        return f;
    }

    /**
     * Total area of ellipsoid in meters<sup>2</sup>. The area of a polygon encircling a pole can
     * be found by adding ellipsoidArea()/2 to the sum of <i>S12</i> for each side of the polygon.
     *
     * @return total area of ellipsoid in meters<sup>2</sup>.
     */
    public double getEllipsoidArea() {
        return 4 * Math.PI * c2;
    }

    /**
     * This is a reformulation of the geodesic problem. The notation is as follows:
     * - at a general point (no suffix or 1 or 2 as suffix)
     * - phi = latitude
     * - beta = latitude on auxiliary sphere
     * - omega = longitude on auxiliary sphere
     * - lambda = longitude
     * - alpha = azimuth of great circle
     * - sigma = arc length along great circle
     * - s = distance
     * - tau = scaled distance (= sigma at multiples of pi/2)
     * - at northwards equator crossing
     * - beta = phi = 0
     * - omega = lambda = 0
     * - alpha = alpha0
     * - sigma = s = 0
     * - a 12 suggix means a difference, e.g., s12 = s2 - s1.
     * - s and c prefixes mean sin and cos
     *
     * @param sinp sinus of p
     * @param sinx sinus of x
     * @param cosx cosinus of x
     * @param c    c
     * @return sin cos series.
     */
    protected static double sinCosSeries(final boolean sinp, final double sinx, final double cosx, final double[] c) {
        // evaluate
        // y = sinp ? sum(c[i] * sin(2 * i * x), i, 1, n) :
        //       sum(c[i] * cos((2 * i + 1) * x), i, 0, n - 1)
        // using Clenshaw summation. N.B. c[0] is unused for sin series
        // Approx operation count = (n + 5) mult and (2 * n + 2) add

        // point to one beyond last element
        var k = c.length;
        var n = k - (sinp ? 1 : 0);

        // 2 * cos(2 * x)
        final var ar = 2 * (cosx - sinx) * (cosx + sinx);
        // accumulators for sum
        var y0 = (n & 1) != 0 ? c[--k] : 0;
        var y1 = 0.0;

        // now n is even
        n /= 2;
        while (n-- != 0) {
            // unroll loop x 2, so accumulators return to their original role
            y1 = ar * y0 - y1 + c[--k];
            y0 = ar * y1 - y0 + c[--k];
        }

        // sin(2 * x) * y0 or cos(x) * (y0 - y1)
        return sinp ? 2 * sinx * cosx * y0 : cosx * (y0 - y1);
    }

    protected double a3f(final double eps) {
        // evaluate a3
        return GeoMath.polyval(NA3 - 1, a3x, 0, eps);
    }

    protected void c3f(final double eps, final double[] c) {
        // evaluate c3 coeffs
        // elements c[1] thru c[NC3 - 1] are set
        var mult = 1.0;
        var o = 0;

        // 1 is index of c3[l]
        for (var l = 1; l < NC3; ++l) {
            // order of polynomial in eps
            final var m = NC3 - l - 1;
            mult *= eps;
            c[l] = mult * GeoMath.polyval(m, c3x, o, eps);
            o += m + 1;
        }
    }

    protected void c4f(final double eps, final double[] c) {
        // evaluate c4 coeffs
        // elements c[0] thru c[NC4 - 1] are set
        var mult = 1.0;
        var o = 0;

        // l is index of c4[l]
        for (var l = 0; l < NC4; ++l) {
            // order of polynomial in eps
            final var m = NC4 - l - 1;
            c[l] = mult * GeoMath.polyval(m, c4x, o, eps);
            o += m + 1;
            mult *= eps;
        }
    }

    // the scale factor a1 - 1 = mean value of (d/dsigma) i1 - 1
    protected static double a1m1f(final double eps) {
        final double[] coeffs = {
                // (1 - eps) * a1 - 1, polynomial in eps2 of order3
                1, 4, 64, 0, 256,
        };
        final var m = NA1 / 2;
        final var t = GeoMath.polyval(m, coeffs, 0, GeoMath.sq(eps)) / coeffs[m + 1];
        return (t + eps) / (1 - eps);
    }

    // The coefficients c1[l] in the Fourier expansion of b1
    protected static void c1f(final double eps, final double[] c) {
        final double[] coeff = {
                // c1[1]/eps^1, polynomial in eps2 of order 2
                -1, 6, -16, 32,
                // c1[2]/eps^2, polynomial in eps2 of order 2
                -9, 64, -128, 2048,
                // c1[3]/eps^3, polynomial in eps2 of order 1
                9, -16, 768,
                // c1[4]/eps^4, polynomial in eps2 of order 1
                3, -5, 512,
                // c1[5]/eps^5, polynomial in eps2 of order 0
                -7, 1280,
                // c1[6]/eps^6, polynomial in eps2 of order 0
                -7, 2048,
        };

        final var eps2 = GeoMath.sq(eps);
        var d = eps;
        var o = 0;
        for (var l = 1; l <= NC1; ++l) {
            // l is index of c1p[l]
            // order of polynomial in eps^2
            final var m = (NC1 - l) / 2;
            c[l] = d * GeoMath.polyval(m, coeff, o, eps2) / coeff[o + m + 1];
            o += m + 2;
            d *= eps;
        }
    }

    // The coefficients c1p[l] in the Fourier expansion of b1p
    protected static void c1pf(final double eps, final double[] c) {
        final double[] coeff = {
                // c1p[l]/eps^1, polynomial in eps2 of order 2
                205, -432, 768, 1536,
                // c1p[2]/eps^2, polynomial in eps2 of order 2
                4005, -4736, 3840, 12288,
                // c1p[3]/eps^3, polynomial in eps2 of order 1
                -225, 116, 384,
                // c1p[4]/eps^4, polynomial in eps2 of order 1
                -7173, 2695, 7680,
                // c1p[5]/eps^5, polynomial in eps2 of order 0
                3467, 7680,
                // c1p[6]/eps^6, polynomial in eps2 of order 0
                38081, 61440,
        };

        final var eps2 = GeoMath.sq(eps);
        var d = eps;
        var o = 0;

        // l is index of c1p[l]
        for (var l = 1; l <= NC1P; ++l) {
            // order of polynomial in eps^2
            final var m = (NC1P - l) / 2;
            c[l] = d * GeoMath.polyval(m, coeff, o, eps2) / coeff[o + m + 1];
            o += m + 2;
            d *= eps;
        }
    }

    // the scale factor a2 - 1 = mean value of (d/dsigma)i2 - 1
    protected static double a2m1f(final double eps) {
        final double[] coeff = {
                // (eps + 1)*a2 - 1, polynomial in eps2 of order 3
                -11, -28, -192, 0, 256,
        };

        final var m = NA2 / 2;
        final var t = GeoMath.polyval(m, coeff, 0, GeoMath.sq(eps)) / coeff[m + 1];
        return (t - eps) / (1 + eps);
    }

    // the coefficients c2[l] in the Fourier expansion of b2
    protected static void c2f(final double eps, final double[] c) {
        final double[] coeff = {
                // c2[1]/eps^1, polynomial in eps2 of order 2
                1, 2, 16, 32,
                // c2[2]/eps^2, polynomial in eps2 of order 2
                35, 64, 384, 2048,
                // c2[3]/eps^3, polynomial in eps2 of order 1
                15, 80, 768,
                // c2[4]/eps^4, polynomial in eps2 of order 1
                7, 35, 512,
                // c2[5]/eps^5, polynomial in eps2 of order 0
                63, 1280,
                // c2[6]/eps^6, polynomial in eps2 of order 0
                77, 2048,
        };

        final var eps2 = GeoMath.sq(eps);
        var d = eps;
        var o = 0;

        // l is index of c2[l]
        for (var l = 1; l <= NC2; ++l) {
            // order of polynomial in eps^2
            final var m = (NC2 - l) / 2;
            c[l] = d * GeoMath.polyval(m, coeff, o, eps2) / coeff[o + m + 1];
            o += m + 2;
            d *= eps;
        }
    }

    // The scale factor a3 = mean value of (d/dsigma)i3
    private void a3coeff() {
        final double[] coeff = {
                // a3, coeff of eps^5, polynomial in n of order 0
                -3, 128,
                // a3, coeff of eps^4, polynomial in n of order 1
                -2, -3, 64,
                // a3, coeff of eps^3, polynomial in n of order 2
                -1, -3, -1, 16,
                // a3, coeff of eps^2, polynomial in n of order 2
                3, -1, -2, 8,
                // a3, coeff of eps^1, polynomial in n of order 1
                1, -1, 2,
                // a3, coeff of eps^0, polynomial in n of order 0
                1, 1,
        };

        var o = 0;
        var k = 0;

        // coeff of eps^j
        for (var j = NA3 - 1; j >= 0; --j) {
            // order of polynomial in n
            final var m = Math.min(NA3 - j - 1, j);
            a3x[k++] = GeoMath.polyval(m, coeff, o, n) / coeff[o + m + 1];
            o += m + 2;
        }
    }

    // The coefficients c3[l] in the Fourier expansion of b3
    private void c3coeff() {
        final double[] coeff = {
                // c3[1], coeff of eps^5, polynomial in n of order 0
                3, 128,
                // c3[1], coeff of eps^4, polynomial in n of order 1
                2, 5, 128,
                // c3[1], coeff of eps^3, polynomial in n of order 2
                -1, 3, 3, 64,
                // c3[1], coeff of eps^2, polynomial in n of order 2
                -1, 0, 1, 8,
                // c3[1], coeff of eps^1, polynomial in n of order 1
                -1, 1, 4,
                // c3[2], coeff of eps^5, polynomial in n of order 0
                5, 256,
                // c3[2], coeff of eps^4, polynomial in n of order 1
                1, 3, 128,
                // c3[2], coeff of eps^3, polynomial in n of order 2
                -3, -2, 3, 64,
                // c3[2], coeff of eps^2, polynomial in n of order 2
                1, -3, 2, 32,
                // c3[3], coeff of eps^5, polynomial in n of order 0
                7, 512,
                // c3[3], coeff of eps^4, polynomial in n of order 1
                -10, 9, 384,
                // c3[3], coeff of eps^3, polynomial in n of order 2
                5, -9, 5, 192,
                // c3[4], coeff of eps^5, polynomial in n of order 0
                7, 512,
                // c3[4], coeff of eps^4, polynomial in n of order 1
                -14, 7, 512,
                // c3[5], coeff of eps^5, polynomial in n of order 0
                21, 2560,
        };

        var o = 0;
        var k = 0;
        // l is index of c3[l]
        for (var l = 1; l < NC3; ++l) {
            for (var j = NC3 - 1; j >= l; --j) {
                // coeff of eps^j
                // order of polynomial in n
                final var m = Math.min(NC3 - j - 1, j);
                c3x[k++] = GeoMath.polyval(m, coeff, o, n) / coeff[o + m + 1];
                o += m + 2;
            }
        }
    }

    private void c4coeff() {
        final double[] coeff = {
                // c4[0], coeff of eps^5, polynomial in n of order 0
                97, 15015,
                // c4[0], coeff of eps^4, polynomial in n of order 1
                1088, 156, 45045,
                // c4[0], coeff of eps^3, polynomial in n of order 2
                -224, -4784, 1573, 45045,
                // c4[0], coeff of eps^2, polynomial in n of order 3
                -10656, 14144, -4576, -858, 45045,
                // c4[0], coeff of eps^1, polynomial in n of order 4
                64, 624, -4576, 6864, -3003, 15015,
                // c4[0], coeff of eps^0, polynomial in n of order 5
                100, 208, 572, 3432, -12012, 30030, 45045,
                // c4[1], coeff of eps^5, polynomial in n of order 0
                1, 9009,
                // c4[1], coeff of eps^4, polynomial in n of order 1
                -2944, 468, 135135,
                // c4[1], coeff of eps^3, polynomial in n of order 2
                5792, 1040, -1287, 135135,
                // c4[1], coeff of eps^2, polynomial in n of order 3
                5952, -11648, 9152, -2574, 135135,
                // c4[1], coeff of eps^1, polynomial in n of order 4
                -64, -624, 4576, -6864, 3003, 135135,
                // c4[2], coeff of eps^5, polynomial in n of order 0
                8, 10725,
                // c4[2], coeff of eps^4, polynomial in n of order 1
                1856, -936, 225225,
                // c4[2], coeff of eps^3, polynomial in n of order 2
                -8448, 4992, -1144, 225225,
                // c4[2], coeff of eps^2, polynomial in n of order 3
                -1440, 4160, -4576, 1716, 225225,
                // c4[3], coeff of eps^5, polynomial in n of order 0
                -136, 63063,
                // c4[3], coeff of eps^4, polynomial in n of order 1
                1024, -208, 105105,
                // c4[3], coeff of eps^3, polynomial in n of order 2
                3584, -3328, 1144, 315315,
                // c4[4], coeff of eps^5, polynomial in n of order 0
                -128, 135135,
                // c4[4], coeff of eps^4, polynomial in n of order 1
                -2560, 832, 405405,
                // c4[5], coeff of eps^5, polynomial in n of order 0
                128, 99099,
        };
        var o = 0;
        var k = 0;
        // l is index of c3[l]
        for (var l = 0; l < NC4; ++l) {
            // coeff of eps^j
            for (var j = NC4 - 1; j >= l; --j) {
                // order of polynomial in n
                final var m = NC4 - j - 1;
                c4x[k++] = GeoMath.polyval(m, coeff, o, n) / coeff[o + m + 1];
                o += m + 2;
            }
        }
    }

    private InverseData inverseInt(
            double lat1, final double lon1, double lat2, final double lon2, final int outmask) {
        final var result = new InverseData();
        final var r = result.g;

        // Compute longitude difference (angDiff does this carefully). Result is in [-180, 180] but
        // -180 is only for west-going geodesics. 180 is for east-going and meridional geodesics.
        lat1 = GeoMath.latFix(lat1);
        lat2 = GeoMath.latFix(lat2);
        r.setLat1(lat1);
        r.setLat2(lat2);

        // if really close to the equator, treat as on equator
        lat1 = GeoMath.angRound(lat1);
        lat2 = GeoMath.angRound(lat2);

        var p = GeoMath.angDiff(lon1, lon2);
        var lon12 = p.getFirst();
        var lon12s = p.getSecond();

        if ((outmask & GeodesicMask.LONG_UNROLL) != 0) {
            r.setLon1(lon1);
            r.setLon2((lon1 + lon12) + lon12s);
        } else {
            r.setLon1(GeoMath.angNormalize(lon1));
            r.setLon2(GeoMath.angNormalize(lon2));
        }

        // make longitude difference positive
        var lonsign = lon12 >= 0 ? 1 : -1;

        // if very close to being on the same half-meridian, then make it so
        lon12 = lonsign * GeoMath.angRound(lon12);
        lon12s = GeoMath.angRound((180 - lon12) - lonsign * lon12s);
        final var lam12 = Math.toRadians(lon12);

        p = GeoMath.sincosd(lon12 > 90 ? lon12s : lon12);
        var slam12 = p.getFirst();
        var clam12 = (lon12 > 90 ? -1 : 1) * p.getSecond();

        // swap points so that point with higher (abs) latitude is point 1
        // if one latitude is a nan, then it becomes lat1
        final var swapp = Math.abs(lat1) < Math.abs(lat2) ? -1 : 1;
        if (swapp < 0) {
            lonsign *= -1;

            final double t = lat1;
            lat1 = lat2;
            lat2 = t;
        }

        // make lat1 <= 0
        final var latsign = lat1 < 0 ? 1 : -1;
        lat1 *= latsign;
        lat2 *= latsign;

        // now we have
        // 0 <= lon2 <= 180
        // -90 <= lat1 <= 0
        // lat1 <= lat2 <= -lat1
        // longsign, swapp, latsign register the transformation to bring the coordinates to this
        // canonical form. In all cases, 1 means no change was made. We make these transformations
        // so that there are few cases to check, e.g., on verifying quadrants in atan2. In addition,
        // this enforces some symmetries in the results returned.

        var m12x = Double.NaN;
        var s12x = Double.NaN;

        p = GeoMath.sincosd(lat1);
        var sbet1 = f1 * p.getFirst();
        var cbet1 = p.getSecond();

        // ensure cbet1 = +epsilon at poles; doing the fix on beta means that sig12 will be <= 2*tiny
        // for two points at the same pole.
        p = GeoMath.norm(sbet1, cbet1);
        sbet1 = p.getFirst();
        cbet1 = p.getSecond();

        cbet1 = Math.max(TINY, cbet1);

        p = GeoMath.sincosd(lat2);
        var sbet2 = f1 * p.getFirst();
        var cbet2 = p.getSecond();

        // ensure cbet2 = +epsilon at poles
        p = GeoMath.norm(sbet2, cbet2);
        sbet2 = p.getFirst();
        cbet2 = p.getSecond();

        cbet2 = Math.max(TINY, cbet2);

        // if cbet1 < -sbet1, then cbet2 - cbet1 is a sensitive measure of the |bet1| - |bet2|.
        // Alternatively (cbet1 >= -sbet1), abs(sbet2) + sbet1 is a better measure.
        // This logic is used in assigning calp2 in lambda12.
        // Sometimes these quantities vanish and in that case we force bet2 = +/- bet1 exactly.
        // An example where it is necessary is the inverse problem
        // 48.522876735459 0 -48.522876735458293 179.599720456223079643

        if (cbet1 < -sbet1) {
            if (cbet2 == cbet1) {
                sbet2 = sbet2 < 0 ? sbet1 : -sbet1;
            }
        } else {
            if (Math.abs(sbet2) == -sbet1) {
                cbet2 = cbet1;
            }
        }

        final var dn1 = Math.sqrt(1.0 + ep2 * GeoMath.sq(sbet1));
        final var dn2 = Math.sqrt(1.0 + ep2 * GeoMath.sq(sbet2));

        double sig12;
        var calp1 = Double.NaN;
        var salp1 = Double.NaN;
        var calp2 = Double.NaN;
        var salp2 = Double.NaN;
        var a12 = Double.NaN;

        // index zero elements of these arrays are unused
        final var c1a = new double[NC1 + 1];
        final var c2a = new double[NC2 + 1];
        final var c3a = new double[NC3];

        var meridian = lat1 == -90 || slam12 == 0;

        if (meridian) {
            // endpoints are on a single full meridian, so the geodesic might lie on a meridian

            // head to the target longitude
            calp1 = clam12;
            salp1 = slam12;

            // at the target we're heading north
            calp2 = 1;
            salp2 = 0;

            // tan(bet) = tan(sig) * cos(alp)
            //noinspection all
            final var ssig1 = sbet1;
            final var csig1 = calp1 * cbet1;
            final var csig2 = calp2 * cbet2;

            // sig12 = sig2 - sig1
            sig12 = Math.atan2(Math.max(0.0, csig1 * sbet2 - ssig1 * csig2),
                    csig1 * csig2 + ssig1 * sbet2);

            final var v = lengths(n, sig12, ssig1, csig1, dn1, sbet2, csig2, dn2, cbet1, cbet2,
                    outmask | GeodesicMask.DISTANCE | GeodesicMask.REDUCED_LENGTH, c1a, c2a);
            s12x = v.s12b;
            m12x = v.m12b;

            if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                r.setScaleM12(v.m12);
                r.setScaleM21(v.m21);
            }

            // add the check for sig12 since zero length geodesics might yield m12 < 0. Test case was
            // echo 20.001 0 20.001 0 | GeodSolve -i
            // in fact, we will have sig12 > pi/2 for meridional geodesic which is not the shortest path.
            if (sig12 < 1 || m12x >= 0) {
                // need at least 2, to handle 90 0 90 180
                if (sig12 < 3 * TINY) {
                    sig12 = m12x = s12x = 0;
                }
                m12x *= b;
                s12x *= b;
                a12 = Math.toDegrees(sig12);
            } else {
                // m12 < 0, i.e., prolate and too close to anti-podal
                meridian = false;
            }
        }

        var omg12 = Double.NaN;
        var somg12 = 2.0;
        var comg12 = Double.NaN;
        // and sbet2 == 0 mimic the way Lambda12 works with calp1 = 0
        if (!meridian && sbet1 == 0.0 && (f <= 0.0 || lon12s >= f * 180.0)) {
            // geodesic runs along equator
            calp1 = calp2 = 0.0;
            salp1 = salp2 = 1.0;
            s12x = a * lam12;
            sig12 = omg12 = lam12 / f1;
            m12x = b * Math.sin(sig12);
            if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                final var value = Math.cos(sig12);
                r.setScaleM12(value);
                r.setScaleM21(value);
            }
            a12 = lon12 / f1;

        } else if (!meridian) {
            // now point1 and point2 belong within a hemisphere bounded by a
            // meridian and geodesic is neither meridional nor equatorial

            // figure a starting point for Newton's method
            final var iv = inverseStart(sbet1, cbet1, dn1, sbet2, cbet2, dn2, lam12, slam12, clam12, c1a, c2a);
            sig12 = iv.sig12;
            salp1 = iv.salp1;
            calp1 = iv.calp1;
            salp2 = iv.salp2;
            calp2 = iv.calp2;
            var dnm = iv.dnm;

            if (sig12 >= 0) {
                // short lines (inverseStart sets salp2, calp2, dnm)
                s12x = sig12 * b * dnm;
                m12x = GeoMath.sq(dnm) * b * Math.sin(sig12 / dnm);
                if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                    final var value = Math.cos(sig12 / dnm);
                    r.setScaleM12(value);
                    r.setScaleM21(value);
                }
                a12 = Math.toDegrees(sig12);
                omg12 = lam12 / (f1 * dnm);
            } else {
                // Newton's method. This is a straightforward solution of f(alp1) =
                // lambda12(alp1) - lam12 = 0 with one wrinkle. f(alp) has exactly one
                // root in the interval (0, pi) and its derivative is positive at the
                // root. Thus f(alp) is positive for alp > alp1 and negative for alp < alp1.
                // During the course of the iteration, a range (alp1a, alp1b) is maintained
                // which brackets the root and with each evaluation of f(alp) the range is
                // shrunk, if possible. Newton's method is restarted whenever the derivative
                // of f is negative (because the new value of alp1 is then further from the
                // solution) or if the new estimate of alp1 lies outside (0,pi); in this
                // case, the new starting guess is taken to be (alp1a + alp1b) / 2.
                var ssig1 = Double.NaN;
                var csig1 = Double.NaN;
                var ssig2 = Double.NaN;
                var csig2 = Double.NaN;
                var eps = Double.NaN;
                var domg12 = Double.NaN;
                var numit = 0;
                // bracketing range
                var salp1a = TINY;
                var calp1a = 1.0;
                var salp1b = TINY;
                var calp1b = -1.0;
                for (boolean tripn = false, tripb = false; numit < MAXIT2; ++numit) {
                    // the WGS84 test set: mean = 1.47, sd = 1.25, max = 16
                    // WGS84 and random input: mean = 2.85, sd = 0.60
                    final double v;
                    final double dv;

                    final var w = lambda12(sbet1, cbet1, dn1, sbet2, cbet2, dn2, salp1, calp1, slam12, clam12,
                            numit < MAXIT1, c1a, c2a, c3a);
                    v = w.lam12;
                    salp2 = w.salp2;
                    calp2 = w.calp2;
                    sig12 = w.sig12;
                    ssig1 = w.ssig1;
                    csig1 = w.csig1;
                    ssig2 = w.ssig2;
                    csig2 = w.csig2;
                    eps = w.eps;
                    domg12 = w.domg12;
                    dv = w.dlam12;

                    // 2 * TOL0 is approximately 1 ulp for a number in [0, pi].
                    // reversed test to allow escape with NaNs
                    final var tmp = (tripn ? 8 : 1);
                    if (tripb || !(Math.abs(v) >= tmp * TOL0)) {
                        break;
                    }

                    // update bracketing values
                    if (v > 0 && (numit > MAXIT1 || calp1 / salp1 > calp1b / salp1b)) {
                        salp1b = salp1;
                        calp1b = calp1;
                    } else if (v < 0 && (numit > MAXIT1 || calp1 / salp1 < calp1a / salp1a)) {
                        salp1a = salp1;
                        calp1a = calp1;
                    }

                    if (numit < MAXIT1 && dv > 0) {
                        final var dalp1 = -v / dv;
                        final var sdalp1 = Math.sin(dalp1);
                        final var cdalp1 = Math.cos(dalp1);
                        final var nsalp1 = salp1 * cdalp1 + calp1 * sdalp1;
                        if (nsalp1 > 0 && Math.abs(dalp1) < Math.PI) {
                            calp1 = calp1 * cdalp1 - salp1 * sdalp1;
                            salp1 = nsalp1;

                            p = GeoMath.norm(salp1, calp1);
                            salp1 = p.getFirst();
                            calp1 = p.getSecond();

                            // in some regimes we don't get quadratic convergence because
                            // slope -> 0. So use convergence conditions based on epsilon
                            // instead of sqrt(epsilon).
                            tripn = Math.abs(v) <= 16 * TOL0;
                            continue;
                        }
                    }

                    // either dv was not positive or updated value was outside legal range.
                    // Use the midpoint of the bracket as the next estimate.
                    // This mechanism is not needed for the WGS84 ellipsoid, but it does
                    // catch problems with more eccentric ellipsoids. Its efficacy is such
                    // for the WGS84 test set with the starting guess set to alp1 = 90deg
                    // the WGS84 test set: mean = 5.21, sd = 3.93, max = 24
                    // WGS84 and random input: mean = 4.74, sd = 0.99
                    salp1 = (salp1a + salp1b) / 2;
                    calp1 = (calp1a + calp1b) / 2;

                    p = GeoMath.norm(salp1, calp1);
                    salp1 = p.getFirst();
                    calp1 = p.getSecond();

                    tripn = false;
                    tripb = (Math.abs(salp1a - salp1) + (calp1a - calp1) < TOLB
                            || Math.abs(salp1 - salp1b) + (calp1 - calp1b) < TOLB);
                }

                // ensure that the reduced length and geodesic scale are computed in a
                // "canonical" way, with the I2 integral.
                final var lengthmask = outmask | ((outmask & (GeodesicMask.REDUCED_LENGTH
                        | GeodesicMask.GEODESIC_SCALE)) != 0 ? GeodesicMask.DISTANCE : GeodesicMask.NONE);
                final var v = lengths(eps, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2, cbet1, cbet2, lengthmask, c1a,
                        c2a);
                s12x = v.s12b;
                m12x = v.m12b;
                if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                    r.setScaleM12(v.m12);
                    r.setScaleM21(v.m21);
                }

                m12x *= b;
                s12x *= b;
                a12 = Math.toDegrees(sig12);
                if ((outmask & GeodesicMask.AREA) != 0) {
                    // omg12 = lam12 - domg12
                    final var sdomg12 = Math.sin(domg12);
                    final var cdomg12 = Math.cos(domg12);
                    somg12 = slam12 * cdomg12 - clam12 * sdomg12;
                    comg12 = clam12 * cdomg12 + slam12 * sdomg12;
                }
            }
        }

        if ((outmask & GeodesicMask.DISTANCE) != 0) {
            // convert -0 to 0
            r.setS12(0.0 + s12x);
        }

        if ((outmask & GeodesicMask.REDUCED_LENGTH) != 0) {
            // convert -0 to 0
            r.setM12(0.0 + m12x);
        }

        if ((outmask & GeodesicMask.AREA) != 0) {
            // from lambda12: sin(alp1) * cos(bet1) = sin(alp0)
            // calp0 > 0
            final var salp0 = salp1 * cbet1;
            final var calp0 = GeoMath.hypot(calp1, salp1 * sbet1);
            final double alp12;
            if (calp0 != 0 && salp0 != 0) {
                // from lambda12: tan(bet) = tan(sig) * cos(alp)
                // multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0).
                var ssig1 = sbet1;
                var csig1 = calp1 * cbet1;
                var ssig2 = sbet2;
                var csig2 = calp2 * cbet2;
                final var k2 = GeoMath.sq(calp0) * ep2;
                final var eps = k2 / (2 * (1 + Math.sqrt(1 + k2)) + k2);
                final var a4 = GeoMath.sq(a) * calp0 * salp0 * e2;

                p = GeoMath.norm(ssig1, csig1);
                ssig1 = p.getFirst();
                csig1 = p.getSecond();

                p = GeoMath.norm(ssig2, csig2);
                ssig2 = p.getFirst();
                csig2 = p.getSecond();

                final var c4a = new double[NC4];
                c4f(eps, c4a);
                final var b41 = sinCosSeries(false, ssig1, csig1, c4a);
                final var b42 = sinCosSeries(false, ssig2, csig2, c4a);
                r.setAreaS12(a4 * (b42 - b41));
            } else {
                // avoid problems with indeterminate sig1, sig2 on equator
                r.setAreaS12(0.0);
            }

            if (!meridian && somg12 > 1) {
                somg12 = Math.sin(omg12);
                comg12 = Math.cos(omg12);
            }

            // long difference not too big
            // lat difference not too big
            if (!meridian && comg12 > -0.7071 && sbet2 - sbet1 < 1.75) {
                // use tan(gamma/2) = tan(omg12/2) * (tan(bet1/2)+tan(bet2/2))/
                // (1+tan(bet1/2)*tan(bet2/2)) with tan(x/2) = sin(x)/(1 + cos(x))
                final var domg12 = 1.0 + comg12;
                final var dbet1 = 1.0 + cbet1;
                final var dbet2 = 1.0 + cbet2;
                alp12 = 2.0 * Math.atan2(somg12 * (sbet1 * dbet2 + sbet2 * dbet1),
                        domg12 * (sbet1 * sbet2 + dbet1 * dbet2));
            } else {
                // alp12 = alp2 - alp1, used in atan2 so no need to normalize
                var salp12 = salp2 * calp1 - calp2 * salp1;
                var calp12 = calp2 * calp1 + salp2 * salp1;

                // the right thing appears to happen if alp1 = +/-180 and alp2 = 0, viz
                // salp12 = -0 and alp12 = -180. However, this depends on the sign
                // being attached to 0 correctly. The following ensures the correct
                // behavior.
                if (salp12 == 0.0 && calp12 < 0.0) {
                    salp12 = TINY * calp1;
                    calp12 = -1.0;
                }
                alp12 = Math.atan2(salp12, calp12);
            }
            r.setAreaS12(r.getAreaS12() + c2 * alp12);
            r.setAreaS12(r.getAreaS12() * swapp * lonsign * latsign);

            // convert -0 to 0
            r.setAreaS12(r.getAreaS12() + 0.0);
        }

        // convert calp, salp to azimuth accounting for lonsign, swapp, latsign
        if (swapp < 0) {
            var t = salp1;
            salp1 = salp2;
            salp2 = t;

            t = calp1;
            calp1 = calp2;
            calp2 = t;

            if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                t = r.getScaleM12();
                r.setScaleM12(r.getScaleM21());
                r.setScaleM21(t);
            }
        }

        salp1 *= swapp * lonsign;
        calp1 *= swapp * latsign;
        salp2 *= swapp * lonsign;
        calp2 *= swapp * latsign;

        // returned value in [0, 180]
        r.setA12(a12);
        result.salp1 = salp1;
        result.calp1 = calp1;
        result.salp2 = salp2;
        result.calp2 = calp2;
        return result;
    }

    /**
     * Safely creates an ellipsoid with.
     *
     * @param a equatorial radius (meters).
     * @param f flattening of ellipsoid. Setting <i>f</i> = 0 gives a sphere. Negative <i>f</i> gives a prolate
     *          ellipsoid.
     * @return a new Geodesic instance or null if something fails.
     */
    @SuppressWarnings("SameParameterValue")
    private static Geodesic safeInstance(final double a, final double f) {
        try {
            return new Geodesic(a, f);
        } catch (final GeodesicException e) {
            return null;
        }
    }

    private LengthsV lengths(
            final double eps, final double sig12, final double ssig1, final double csig1,
            final double dn1, final double ssig2, final double csig2, final double dn2,
            final double cbet1, final double cbet2,
            // scratch areas of the right size
            int outmask, final double[] c1a, final double[] c2a) {
        // return m12b = (reduced length)/mB; also calculate s12b = distance/mB,
        // and m0 = coefficient of secular term in expression for reduced length.
        outmask &= GeodesicMask.OUT_MASK;

        // to hold s12b, m12b, m0, M12, M21
        final var v = new LengthsV();

        var m0x = 0.0;
        var j12 = 0.0;
        var a1 = 0.0;
        var a2 = 0.0;
        if ((outmask & (GeodesicMask.DISTANCE | GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE)) != 0) {
            a1 = a1m1f(eps);
            c1f(eps, c1a);
            if ((outmask & (GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE)) != 0) {
                a2 = a2m1f(eps);
                c2f(eps, c2a);
                m0x = a1 - a2;
                a2 = 1 + a2;
            }
            a1 = 1 + a1;
        }

        if ((outmask & GeodesicMask.DISTANCE) != 0) {
            final var b1 = sinCosSeries(true, ssig2, csig2, c1a) - sinCosSeries(true, ssig1, csig1, c1a);
            // missing a factor of mB
            v.s12b = a1 * (sig12 + b1);
            if ((outmask & (GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE)) != 0) {
                final var b2 = sinCosSeries(true, ssig2, csig2, c2a) - sinCosSeries(true, ssig1, csig1, c2a);
                j12 = m0x * sig12 + (a1 * b1 - a2 * b2);
            }
        } else if ((outmask & (GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE)) != 0) {
            // assume here that NC1 >= NC2
            for (var l = 1; l <= NC2; ++l) {
                c2a[l] = a1 * c1a[l] - a2 * c2a[l];
            }
            j12 = m0x * sig12 + (sinCosSeries(true, ssig2, csig2, c2a)
                    - sinCosSeries(true, ssig1, csig1, c2a));
        }

        if ((outmask & GeodesicMask.REDUCED_LENGTH) != 0) {
            v.m0 = m0x;
            // Missing a factor of mB.
            // Add parens around (csig1 * ssig2) and (ssig1 * csig2) to ensure
            // accurate cancellation in the case of coincident points
            v.m12b = dn2 * (csig1 * ssig2) - dn1 * (ssig1 * csig2) - csig1 * csig2 * j12;
        }

        if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
            final var csig12 = csig1 * csig2 + ssig1 * ssig2;
            final var t = ep2 * (cbet1 - cbet2) * (cbet1 + cbet2) / (dn1 + dn2);
            v.m12 = csig12 + (t * ssig2 - csig2 * j12) * ssig1 / dn1;
            v.m21 = csig12 - (t * ssig1 - csig1 * j12) * ssig2 / dn2;
        }
        return v;
    }

    private static double astroid(final double x, final double y) {
        // solve k^4 + 2 * k^3 - (x^2 + y^2 - 1) * k^2 - 2 * y^2 * k - y^2 = 0 for positive root k.
        // this solution is adapted from Geocentric.reverse
        final double k;
        final var p = GeoMath.sq(x);
        //noinspection all
        final var q = GeoMath.sq(y);
        final var r = (p + q - 1) / 6;

        if (!(q == 0 && r <= 0)) {
            // avoid possible division by zero when r = 0 by multiplying equations for s and t by
            // r^3 and r, resp.

            // s = r^3 * s
            final var s = p * q / 4;
            final var r2 = GeoMath.sq(r);
            final var r3 = r * r2;
            // the discriminant of the quadratic equation for T3. This is zero on the
            // evolute curve p^(1/3) + q^(1/3) = 1
            final var disc = s * (s + 2 * r3);
            var u = r;
            if (disc >= 0) {
                var t3 = s + r3;

                // pick the sign on the sqrt to maximize abs(t3). This minimizes loss of precision
                // due to cancellation. The result is unchanged because of the way the T is used in
                // definition of u.

                // t3 = (r * t)^3
                t3 += t3 < 0 ? -Math.sqrt(disc) : Math.sqrt(disc);

                // N.B. cbrt always returns the double root. cbrt(-8) = -2.
                // t = r * t
                final var t = GeoMath.cbrt(t3);

                // t can be zero; but then r2 / t -> 0.
                u += t + (t != 0 ? r2 / t : 0);
            } else {
                // t is complex, but the way u is defined the result is double.
                final var ang = Math.atan2(Math.sqrt(-disc), -(s + r3));
                // there are three possible cube roots. We choose the root which
                // avoids cancellation. Note that disc < 0 implies that r < 0.
                u += 2 * r * Math.cos(ang / 3);
            }

            // guaranteed positive
            final var v = Math.sqrt(GeoMath.sq(u) + q);
            // avoid loss of accuracy when u < 0
            // u + v, guaranteed positive
            final var uv = u < 0 ? q / (v - u) : u + v;
            // positive?
            final var w = (uv - q) / (2 * v);

            // rearrange expression for k to avoid loss of accuracy due to subtraction. Division
            // by 0 not possible because uv > 0, w >= 0

            // guaranteed positive
            k = uv / (Math.sqrt(uv + GeoMath.sq(w)) + w);
        } else {
            // y = 0 with |x| <= 1. Handle this case directly
            // for y small, positive root is k = abs(y) / sqrt(1 - x ^2)
            k = 0.0;
        }
        return k;
    }

    private InverseStartV inverseStart(
            final double sbet1, final double cbet1, final double dn1,
            final double sbet2, final double cbet2, final double dn2,
            final double lam12, final double slam12, final double clam12,
            // scratch areas of the right size
            final double[] c1a, final double[] c2a) {
        // return a starting point for Newton's method in salp1 and calp1 (function value is -1).
        // If Newton's method doesn't need to be used, return also salp2 and calp2 and function
        // value is sig12.

        // to hold sig12, salp1, calp1, salp2, calp2, dnm.
        final var w = new InverseStartV();

        // return value
        w.sig12 = -1;

        // bet12 = bet2 - bet1 in [0, pi); bet12a = bet2 + bet1 in (-pi, 0]
        final var sbet12 = sbet2 * cbet1 - cbet2 * sbet1;
        final var cbet12 = cbet2 * cbet1 + sbet2 * sbet1;
        final var sbet12a = sbet2 * cbet1 + cbet2 * sbet1;
        final var shortline = cbet12 >= 0 && sbet12 < 0.5 && cbet2 * lam12 < 0.5;
        double somg12;
        double comg12;
        if (shortline) {
            var sbetm2 = GeoMath.sq(sbet1 + sbet2);

            // sin((bet1 + bet2) / 2)^2
            // = (sbet1 + sbet2)^2 / ((sbet1 + sbet2)^2 + (cbet1 + cbet2)^2)
            sbetm2 /= sbetm2 + GeoMath.sq(cbet1 + cbet2);
            w.dnm = Math.sqrt(1 + ep2 * sbetm2);
            final var omg12 = lam12 / (f1 * w.dnm);
            somg12 = Math.sin(omg12);
            comg12 = Math.cos(omg12);
        } else {
            somg12 = slam12;
            comg12 = clam12;
        }

        w.salp1 = cbet2 * somg12;
        w.calp1 = comg12 >= 0.0
                ? sbet12 + cbet2 * sbet1 * GeoMath.sq(somg12) / (1.0 + comg12)
                : sbet12a - cbet2 * sbet1 * GeoMath.sq(somg12) / (1.0 - comg12);

        final var ssig12 = GeoMath.hypot(w.salp1, w.calp1);
        final var csig12 = sbet1 * sbet2 + cbet1 * cbet2 * comg12;

        if (shortline && ssig12 < etol2) {
            // really short lines
            w.salp2 = cbet1 * somg12;
            w.calp2 = sbet12 - cbet1 * sbet2 * (comg12 >= 0.0 ? GeoMath.sq(somg12) / (1.0 + comg12) : 1.0 - comg12);

            final var p = GeoMath.norm(w.salp2, w.calp2);
            w.salp2 = p.getFirst();
            w.calp2 = p.getSecond();

            // set return value
            w.sig12 = Math.atan2(ssig12, csig12);
            // skip astroid calc if too eccentric
        } else if (!(Math.abs(n) > 0.1 || csig12 >= 0 || ssig12 >= 6 * Math.abs(n) * Math.PI * GeoMath.sq(cbet1))) {
            // scale lam12 and bet2 to x, y coordinate system where antipodal point is at origin and
            // singular point is at y = 0, x = -1
            final double y;
            final double lamscale;
            final double betscale;

            // In C++ volatile declaration needed to fix inverse case
            // 56.320923501171 0 -56.320923501171 179.664747671772880215
            final double x;
            // lam12 - pi
            final var lam12x = Math.atan2(-slam12, -clam12);
            if (f >= 0) {
                // in fact f == 0 does not get here

                // x = dlong, y = dlat
                final var k2 = GeoMath.sq(sbet1) * ep2;
                final var eps = k2 / (2 * (1 + Math.sqrt(1 + k2)) + k2);
                lamscale = f * cbet1 * a3f(eps) * Math.PI;

                betscale = lamscale * cbet1;

                x = lam12x / lamscale;
                y = sbet12a / betscale;
            } else {
                // mF < 0
                // x = dlat, y = dlong
                final var cbet12a = cbet2 * cbet1 - sbet2 * sbet1;
                final var bet12a = Math.atan2(sbet12a, cbet12a);
                final double m12b;
                final double m0;

                // in the case of lon12 = 180, this repeats a calculation made in inverse
                final var v = lengths(n, Math.PI + bet12a, sbet1, -cbet1, dn1, sbet2, cbet2, dn2, cbet1, cbet2,
                        GeodesicMask.REDUCED_LENGTH, c1a, c2a);
                m12b = v.m12b;
                m0 = v.m0;

                x = -1 + m12b / (cbet1 * cbet2 * m0 * Math.PI);
                betscale = x < -0.01 ? sbet12a / x : -f * GeoMath.sq(cbet1) * Math.PI;
                lamscale = betscale / cbet1;
                y = lam12x / lamscale;
            }

            if (y > -TOL1 && x > -1 - XTHRESH) {
                // strip near cut
                if (f >= 0) {
                    w.salp1 = Math.min(1.0, -x);
                    w.calp1 = -Math.sqrt(1.0 - GeoMath.sq(w.salp1));
                } else {
                    w.calp1 = Math.max(x > -TOL1 ? 0.0 : -1.0, x);
                    w.salp1 = Math.sqrt(1.0 - GeoMath.sq(w.calp1));
                }
            } else {
                // estimate alp1, by solving the astroid problem

                // could estimate alpha1 = theta + pi/2, directly, i.e.,
                // calp1 = y / k; salp1 = -x / (1 + k); for mF >= 0
                // calp1 = x / (1 + k); salp1 = -y / k; for mF < 0 (need to check)

                // However, it's better to estimate omg12 from astroid and use spherical
                // formula to compute alp1. This reduces the mean number of Newton iterations
                // for astroid cases from 2.24 (min 0, max 6) to 2.12 (min 0 max 5). The changes
                // in the number of iterations are as follows:

                // change    percent
                //   1           5
                //   0           78
                //  -1          16
                //  -2          0.6
                //  -3          0.04
                //  -4          0.002

                // The histogram of iterations is (m = number of iterations estimating alp1 directly,
                // n = number of iterations estimating via omg12, total number of trials = 148605):

                // iter      m       n
                //   0       148     186
                //   1       13046   13845
                //   2       93315   102225
                //   3       36189   32341
                //   4       5396    7
                //   5       455     1
                //   6       56      0

                // Because omg12 is near pi, estimate work with omg12a = pi - omg12
                final var k = astroid(x, y);
                final var omg12a = lamscale * (f >= 0.0 ? -x * k / (1.0 + k) : -y * (1 + k) / k);
                somg12 = Math.sin(omg12a);
                comg12 = -Math.cos(omg12a);

                // update spherical estimate of alp1 using omg12 instead of lam12
                w.salp1 = cbet2 * somg12;
                w.calp1 = sbet12a - cbet2 * sbet1 * GeoMath.sq(somg12) / (1.0 - comg12);
            }
        }

        // sanity check on starting guess. Backwards check allows NaN through
        if (!(w.salp1 <= 0.0)) {
            final var p = GeoMath.norm(w.salp1, w.calp1);
            w.salp1 = p.getFirst();
            w.calp1 = p.getSecond();
        } else {
            w.salp1 = 1;
            w.calp1 = 0;
        }
        return w;
    }

    private Lambda12V lambda12(
            final double sbet1, final double cbet1, final double dn1,
            final double sbet2, final double cbet2, final double dn2,
            final double salp1, double calp1,
            final double slam120, final double clam120, final boolean diffp,
            // scratch areas of the right size
            final double[] c1a, final double[] c2a, final double[] c3a) {
        // object to hold lam12, salp2, calp2, sig12, ssig1, csig1, ssig2, csig2, eps, domg12, dlam12
        final var w = new Lambda12V();

        if (sbet1 == 0 && calp1 == 0) {
            // break degeneracy of equatorial line. This case has already been handled
            calp1 = -TINY;
        }

        // sin(alp1) * cos(bet1) = sin(alp0)
        final var salp0 = salp1 * cbet1;
        // calp0 > 0
        final var calp0 = GeoMath.hypot(calp1, salp1 * sbet1);

        double somg1;
        double comg1;
        double somg2;
        double comg2;
        double somg12;
        double comg12;
        // tan(bet1) = tan(sig1) * cos(alp1)
        // tan(omg1) = sin(alp0) * tan(sig1) = tan(omg1) = tan(alp1) * sin(bet1)
        w.ssig1 = sbet1;
        somg1 = salp0 * sbet1;
        w.csig1 = comg1 = calp1 * cbet1;

        var p = GeoMath.norm(w.ssig1, w.csig1);
        w.ssig1 = p.getFirst();
        w.csig1 = p.getSecond();

        // GeoMath.norm(somg1, comg1); -- don't need to normalize!

        // Enforce symmetries in the case abs(bet2) = -bet1. Need to be careful about this case,
        // since this can yield singularities in the Newton iteration.
        // sin(alp2) * cos(bet2) = sin(alp0)
        w.salp2 = cbet2 != cbet1 ? salp0 / cbet2 : salp1;
        // calp2 = sqrt(1 - sq(salp2))
        //       = sqrt(sq(calp0) - sq(sbet2)) / cbet2
        // and subst for calp0 and rearrange to give (choose positive sqrt
        // to give alp2 in [0, pi/2]).
        final var tmp = (cbet1 < -sbet1 ? (cbet2 - cbet1) * (cbet1 + cbet2) : (sbet1 - sbet2) * (sbet1 + sbet2));
        w.calp2 = cbet2 != cbet1 || Math.abs(sbet2) != -sbet1
                ? Math.sqrt(GeoMath.sq(calp1 * cbet1) + tmp) / cbet2 : Math.abs(calp1);
        // tan(bet2) = tan(sig2) * cos(alp2)
        // tan(omg2) = sin(alp0) * tan(sig2)
        w.ssig2 = sbet2;
        somg2 = salp0 * sbet2;
        w.csig2 = comg2 = w.calp2 * cbet2;

        p = GeoMath.norm(w.ssig2, w.csig2);
        w.ssig2 = p.getFirst();
        w.csig2 = p.getSecond();

        // GeoMath.norm(somg2, comg2); -- don't need to normalize!

        // sig12 = sig2 - sig1, limit to [0, pi]
        w.sig12 = Math.atan2(Math.max(0.0, w.csig1 * w.ssig2 - w.ssig1 * w.csig2),
                w.csig1 * w.csig2 + w.ssig1 * w.ssig2);

        // omg12 = omg2 - omg1, limit to [0, pi]
        somg12 = Math.max(0.0, comg1 * somg2 - somg1 * comg2);
        comg12 = comg1 * comg2 + somg1 * somg2;
        // eta = omg12 - lam120
        final var eta = Math.atan2(somg12 * clam120 - comg12 * slam120, comg12 * clam120 + somg12 * slam120);

        final var k2 = GeoMath.sq(calp0) * ep2;
        w.eps = k2 / (2 * (1 + Math.sqrt(1 + k2)) + k2);
        c3f(w.eps, c3a);
        final var b312 = (sinCosSeries(true, w.ssig2, w.csig2, c3a)
                - sinCosSeries(true, w.ssig1, w.csig1, c3a));
        w.domg12 = -f * a3f(w.eps) * salp0 * (w.sig12 + b312);
        w.lam12 = eta + w.domg12;

        if (diffp) {
            if (w.calp2 == 0) {
                w.dlam12 = -2 * f1 * dn1 / sbet1;
            } else {
                final var v = lengths(w.eps, w.sig12, w.ssig1, w.csig1, dn1, w.ssig2, w.csig2, dn2, cbet1, cbet2,
                        GeodesicMask.REDUCED_LENGTH, c1a, c2a);
                w.dlam12 = v.m12b;
                w.dlam12 *= f1 / (w.calp2 * cbet2);
            }
        }

        return w;
    }

    private static class Lambda12V {
        private double lam12;
        private double salp2;
        private double calp2;
        private double sig12;
        private double ssig1;
        private double csig1;
        private double ssig2;
        private double csig2;
        private double eps;
        private double domg12;
        private double dlam12;

        private Lambda12V() {
            lam12 = salp2 = calp2 = sig12 = ssig1 = csig1 = ssig2 = csig2 = eps = domg12 = dlam12 = Double.NaN;
        }
    }

    private static class InverseStartV {
        private double sig12;
        private double salp1;
        private double calp1;

        // only updated if return value >= 0
        private double salp2;
        private double calp2;

        // only updated for short lines
        private double dnm;

        private InverseStartV() {
            sig12 = salp1 = calp1 = salp2 = calp2 = dnm = Double.NaN;
        }
    }

    private static class LengthsV {
        private double s12b;
        private double m12b;
        private double m0;
        private double m12;
        private double m21;

        private LengthsV() {
            s12b = m12b = m0 = m12 = m21 = Double.NaN;
        }
    }

    private static class InverseData {
        private final GeodesicData g;
        private double salp1;
        private double calp1;
        private double salp2;
        private double calp2;

        private InverseData() {
            g = new GeodesicData();
            salp1 = calp1 = salp2 = calp2 = Double.NaN;
        }
    }
}
