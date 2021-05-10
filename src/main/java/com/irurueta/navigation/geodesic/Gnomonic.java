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
 * Gnomonic projection.
 * Gnomonic projection centered at an arbitrary position <i>C</i> on the ellipsoid. This projection
 * is derived in Section 8 of
 * <ul>
 *     <li>
 *         C. F. F. Karney, <a href="https://doi.org/10.1007/s00190-012-0578-z">Algorithms for
 *         geodesics</a>, J. Geodesy <b>87</b>, 43&ndash;55 (2013);
 *         DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">10.1007/s00190-012-0578-z</a>;
 *     </li>
 * </ul>
 * The gnomonic projection of a point <i>P</i> on the ellipsoid is defined as follows: compute the
 * geodesic line from <i>C</i> to <i>P</i>; compute the reduced length <i>m12</i>, geodesic scale
 * <i>M12</i>, and &rho; = <i>m12</i>/<i>M12</i>; finally, this gives the coordinates <i>x</i> and
 * <i>y</i> of <i>P</i> in gnomonic projection with <i>x</i> = &rho; sin <i>azi1</i>; <i>y</i> = &rho;
 * cos <i>azi1</i>, where <i>azi1</i> is the azimuth of the geodesic at <i>C</i>. The method
 * {@link Gnomonic#forward(double, double, double, double)} performs the forward projection and
 * {@link Gnomonic#reverse(double, double, double, double)} is the inverse of the projection. The
 * methods also return the azimuth <i>azi</i> of the geodesic at <i>P</i> and reciprocal scale
 * <i>rk</i> in the azimuthal direction. The scale in the radial direction is 1/<i>rk</i><sup>2</sup>.
 * For a sphere, &rho; reduces to <i>a</i> tan (<i>s12</i>/<i>a</i>), where <i>s12</i> is the length
 * of the geodesic from <i>C</i> to <i>P</i>, and the gnomonic projection has the property that all
 * geodesics appear as straight lines. For an ellipsoid, this property holds only for geodesics
 * interesting the centers. However geodesic segments close to the center are approximately straight.
 * Consider a geodesic segment of length <i>1</i>. Let <i>T</i> be the point on the geodesic
 * (extended if necessary) closest to <i>C</i>, the center of the projection, and <i>t</i>, be the
 * distance <i>CT</i>. To lowest order, the maximum deviation (as a true distance) of the corresponding
 * gnomonic line segment (i.e., with the same end points) from the geodesic is
 * (<i>K</i>(<i>T</i> &minus; <i>K</i>(<i>C</i>))
 * <i>l</i><sup>2</sup> <i>t</i> / 32.
 * where <i>K</i> is the Gaussian curvature.
 * This result applies for any surface. For an ellipsoid of revolution, consider all geodesics whose end
 * points are within a distance <i>r</i> of <i>C</i>. For a given <i>r</i>, the deviation is maximum
 * when the latitude of <i>C</i> is 46&deg;, when endpoints are a distance <i>r</i> away, and when
 * their azimuths from the center are &plusmn; 45&deg; or &plusmn; 135&deg;. To lowest order in
 * <i>r</i> and the flattening <i>f</i>, the deviation is <i>f</i> (<i>r</i>/2<i>a</i>)<sup>3</sup>
 * <i>r</i>.
 * <b>CAUTION:</b> The definition of this projection for a sphere is standard. However, there is no
 * standard for how it should be extended to an ellipsoid.
 * The choices are:
 * <ul>
 *     <li>
 *         Declare that the projection is undefined for an ellipsoid.
 *     </li>
 *     <li>
 *         Project to a tangent plane from the center of the ellipsoid. This causes great ellipses to
 *         appear as straight lines in the projection; i.e., it generalizes the spherical great circle
 *         to a great ellipse. This was proposed independently by Bowring and Williams in 1997.
 *     </li>
 *     <li>
 *         Project to the conformal sphere with the constant of integration chosen so that the values
 *         of the latitude match for the center point and perform a central projection onto the plane
 *         tangent to the conformal sphere at the center point. This causes normal section through the
 *         center point to appear as straight lines in the projection; i.e., it generalizes the
 *         spherical great circle to a normal section. This was proposed by I. G. Letoval'tsev,
 *         Generalization of the gnomonic projection for a spheroid and the principal geodetic
 *         problems involved in the alignment of surface routes, Geodesy and Aerophotography(5),
 *         271&ndash;275 (1963)
 *     </li>
 *     <li>
 *         The projection given here. This causes geodesics close to the center point to appear as
 *         straight lines in the projection; i.e., it generalizes the spherical great circle to a
 *         geodesic.
 *     </li>
 * </ul>
 * Example of use:
 * <pre>
 * // Example of using the Gnomonic.java class
 * import com.irurueta.navigation.geodesic.Geodesic;
 * import com.irurueta.navigation.geodesic.Gnomonic;
 * import com.irurueta.navigation.geodesic.GnomonicData;
 * public class ExampleGnomonic {
 *   public static void main(String[] args) {
 *     Geodesic geod = Geodesic.WGS84;
 *     double lat0 = 48 + 50 / 60.0, lon0 = 2 + 20 / 60.0; // Paris
 *     Gnomonic gnom = new Gnomonic(geod);
 *     {
 *       // Sample forward calculation
 *       double lat = 50.9, lon = 1.8; // Calais
 *       GnomonicData proj = gnom.Forward(lat0, lon0, lat, lon);
 *       System.out.println(proj.x + &quot; &quot; + proj.y);
 *     }
 *     {
 *       // Sample reverse calculation
 *       double x = -38e3, y = 230e3;
 *       GnomonicData proj = gnom.Reverse(lat0, lon0, x, y);
 *       System.out.println(proj.lat + &quot; &quot; + proj.lon);
 *     }
 *   }
 * }
 * </pre>
 */
public class Gnomonic {
    private static final double EPS = 0.01 * Math.sqrt(GeoMath.EPSILON);
    private static final int NUMIT = 10;

    /**
     * Earth geodesic.
     */
    private final Geodesic mEarth;

    /**
     * Major equatorial Earth radius.
     */
    private final double mA;

    /**
     * Earth flattening.
     */
    private final double mF;

    /**
     * Constructor for Gnomonic.
     *
     * @param earth the {@link Geodesic} object to use for geodesic calculations.
     */
    public Gnomonic(final Geodesic earth) {
        mEarth = earth;
        mA = mEarth.getMajorRadius();
        mF = mEarth.getFlattening();
    }

    /**
     * Forward projection, from geographic to gnomonic.
     * <i>lat0</i> and <i>lat</i> should be in the range [&minus;90&deg;, 90&deg;] and <i>lon0</i>
     * and <i>lon</i> should be in the range [&minus;540&deg;, 540&deg;). The scale of the projection
     * is 1/<i>rk<sup>2</sup></i> in the "radial" direction, <i>azi</i> clockwise from true north,
     * and is 1/<i>rk</i> in the direction perpendicular to this. If the point lies "over the
     * horizon", i.e., if <i>rk</i> &le; 0, then NaNs are returned for <i>x</i> and <i>y</i> (the
     * correct values are returned for <i>azi</i> and <i>rk</i>). A call to forward followed by a
     * call to reverse will return the original (<i>lat</i>, <i>lon</i>) (to within roundoff)
     * provided the point in not over the horizon.
     *
     * @param lat0 latitude of center point of projection (degrees).
     * @param lon0 longitude of center point of projection (degrees).
     * @param lat  latitude of point (degrees).
     * @param lon  longitude of point (degrees).
     * @return {@link GnomonicData} object with the following fields:
     * <i>lat0</i>, <i>lon0</i>, <i>lat</i>, <i>lon</i>, <i>x</i>, <i>y</i>, <i>azi</i>, <i>rk</i>.
     */
    public GnomonicData forward(
            final double lat0, final double lon0, final double lat, final double lon) {
        final GeodesicData inv = mEarth.inverse(lat0, lon0, lat, lon,
                GeodesicMask.AZIMUTH | GeodesicMask.GEODESIC_SCALE |
                        GeodesicMask.REDUCED_LENGTH);
        final GnomonicData fwd = new GnomonicData(lat0, lon0, lat, lon, Double.NaN, Double.NaN,
                inv.getAzi2(), inv.getScaleM12());

        if (inv.getScaleM12() > 0) {
            final double rho = inv.getM12() / inv.getScaleM12();
            final Pair p = GeoMath.sincosd(inv.getAzi1());
            fwd.setX(rho * p.getFirst());
            fwd.setY(rho * p.getSecond());
        }

        return fwd;
    }

    /**
     * Reverse projection, from gnomonic to geographic.
     * <i>lat</i> will be in the range [&minus;90&deg;, 90&deg;] and <i>lon</i> will be in the
     * range (&minus;180&deg;, 180&deg;]. The scale of the projection is 1/<i>rk<sup>2</sup></i>
     * in the "radial" direction, <i>azi</i> clockwise from true north, and is 1/<i>rk</i> in the
     * direction perpendicular to this. Even though all inputs should return a valid <i>lat</i>
     * and <i>lon</i>, it's possible that the procedure fails to converge for very large <i>x</i>
     * or <i>y</i>; in this case NaNs are returned for very large <i>x</i> or <i>y</i>; in this
     * case NaNs are returned for all the output arguments. A call to reverse followed by a call
     * to forward will return the original (<i>x</i>, <i>y</i>) (to round-off).
     *
     * @param lat0 latitude of center point of projection (degrees). <i>lat0</i> should be in the
     *             range [&minus;90&deg;, 90&deg;]
     * @param lon0 longitude of center point of projection (degrees). <i>lon0</i> should be in the
     *             range [&minus;540&deg;, 540&deg;).
     * @param x    easting of point (meters).
     * @param y    northing of point (meters).
     * @return {@link GnomonicData} object with the following fields:
     * <i>lat0</i>, <i>lon0</i>, <i>lat</i>, <i>lon</i>, <i>x</i>, <i>y</i>,
     * <i>azi</i>, <i>rk</i>.
     */
    public GnomonicData reverse(
            final double lat0, final double lon0, final double x, final double y) {
        final GnomonicData rev = new GnomonicData(lat0, lon0, Double.NaN, Double.NaN, x, y,
                Double.NaN, Double.NaN);

        //noinspection all
        final double azi0 = GeoMath.atan2d(x, y);
        double rho = Math.hypot(x, y);
        double s = mA * Math.atan(rho / mA);
        final boolean little = rho <= mA;

        if (!little) {
            rho = 1 / rho;
        }

        final GeodesicLine line = mEarth.line(lat0, lon0, azi0, GeodesicMask.LATITUDE |
                GeodesicMask.LONGITUDE | GeodesicMask.AZIMUTH | GeodesicMask.DISTANCE_IN |
                GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE);

        int count = NUMIT;
        int trip = 0;
        GeodesicData pos = null;

        while (count-- > 0) {
            pos = line.position(s, GeodesicMask.LONGITUDE | GeodesicMask.LATITUDE |
                    GeodesicMask.AZIMUTH | GeodesicMask.DISTANCE_IN |
                    GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE);

            if (trip > 0) {
                break;
            }

            final double ds = little
                    ? ((pos.getM12() / pos.getScaleM12()) - rho) * pos.getScaleM12() * pos.getScaleM12()
                    : (rho - (pos.getScaleM12() / pos.getM12())) * pos.getM12() * pos.getM12();
            s -= ds;

            if (Math.abs(ds) <= EPS * mA) {
                trip++;
            }
        }

        if (trip == 0) {
            return rev;
        }

        rev.setLat(pos.getLat2());
        rev.setLon(pos.getLon2());
        rev.setAzi(pos.getAzi2());
        rev.setRk(pos.getScaleM12());

        return rev;
    }

    /**
     * Gets the equatorial radius of the ellipsoid (meters). This is the value inherited from the
     * Geodesic object used in the constructor.
     *
     * @return <i>a</i> the equatorial radius of the ellipsoid (meters).
     */
    public double getMajorRadius() {
        return mA;
    }

    /**
     * Gets the flattening of the ellipsoid.
     *
     * @return <i>f</i> the flattening of the ellipsoid. This is the value inherited from the
     * Geodesic object used in the constructor.
     */
    public double getFlattening() {
        return mF;
    }
}
