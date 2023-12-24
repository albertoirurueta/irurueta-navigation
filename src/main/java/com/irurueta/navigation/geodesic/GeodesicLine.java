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
 * A geodesic line
 * GeodesicLine facilitates the determination of a series of points on a single geodesic. The
 * starting point (<i>lat1</i>, <i>lon1</i>) and the azimuth <i>azi1</i> are specified in the
 * constructor; alternatively, the {@link Geodesic#line} method can be used to create a GeodesicLine.
 * {@link #position} returns the location of point 2 a distance <i>s12</i> along the geodesic.
 * Alternatively {@link #arcPosition} gives the position of point 2 an arc length <i>a12</i> along
 * the geodesic.
 * You can register the position of a reference point 3 a distance (arc length), <i>s13</i>
 * (<i>a13</i>) along the geodesic with the {@link #setDistance} ({@link #setArc}) functions. Points
 * a fractional distance along the line can be found by providing, for example, 0.5 * {@link #getDistance}
 * as an argument to {@link #position}. The {@link Geodesic#inverseLine} or {@link Geodesic#directLine}
 * methods return GeodesicLine objects with point 3 set to the point 2 of the corresponding geodesic
 * problem. GeodesicLine objects created with the public constructor or with {@link Geodesic#line}
 * have <i>s13</i> and <i>a13</i> set to NaNs.
 * The calculations are accurate to better than 15 nm (15 nanometers). See Sec. 9 of
 * <a href="https://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a> for details. The algorithms used
 * by this class are based on series expansions using the flattening <i>f</i> as a small parameter.
 * These are only accurate for |<i>f</i> &lt; 0.02; however reasonably accurate results will be
 * obtained for |<i>f</i> &lt; 0.2.
 * The algorithms are described in
 * <ul>
 *     <li>
 *         C. F. F. Karney, <a href="https://doi.org/10.1007/s00190-012-0578-z">
 *         Algorithms for geodesics</a>,
 *         J. Geodesy <b>87</b>, 43&ndash;55 (2013)
 *         (<a href="https://geographiclib.sourceforge.io/geod-addenda.html">addenda</a>).
 *     </li>
 * </ul>
 * Here's an example of using this class
 * <pre>
 * {@code
 * import net.sf.geographiclib.*;
 * public class GeodesicLineTest {
 *   public static void main(String[] args) {
 *     // Print waypoints between JFK and SIN
 *     Geodesic geod = Geodesic.WGS84;
 *     double
 *       lat1 = 40.640, lon1 = -73.779, // JFK
 *       lat2 =  1.359, lon2 = 103.989; // SIN
 *     GeodesicLine line = geod.InverseLine(lat1, lon1, lat2, lon2,
 *                                          GeodesicMask.DISTANCE_IN |
 *                                          GeodesicMask.LATITUDE |
 *                                          GeodesicMask.LONGITUDE);
 *     double ds0 = 500e3;     // Nominal distance between points = 500 km
 *     // The number of intervals
 *     int num = (int)(Math.ceil(line.Distance() / ds0));
 *     {
 *       // Use intervals of equal length
 *       double ds = line.Distance() / num;
 *       for (int i = 0; i <= num; ++i) {
 *         GeodesicData g = line.Position(i * ds,
 *                                        GeodesicMask.LATITUDE |
 *                                        GeodesicMask.LONGITUDE);
 *         System.out.println(i + " " + g.lat2 + " " + g.lon2);
 *       }
 *     }
 *     {
 *       // Slightly faster, use intervals of equal arc length
 *       double da = line.Arc() / num;
 *       for (int i = 0; i <= num; ++i) {
 *         GeodesicData g = line.ArcPosition(i * da,
 *                                           GeodesicMask.LATITUDE |
 *                                           GeodesicMask.LONGITUDE);
 *         System.out.println(i + " " + g.lat2 + " " + g.lon2);
 *       }
 *     }
 *   }
 * }}</pre>
 */
public class GeodesicLine {

    private static final int NC1 = Geodesic.NC1;
    private static final int NC1P = Geodesic.NC1P;
    private static final int NC2 = Geodesic.NC2;
    private static final int NC3 = Geodesic.NC3;
    private static final int NC4 = Geodesic.NC4;

    private double mLat1;
    private double mLon1;
    private double mAzi1;

    private double mA;
    private double mF;
    private double mB;
    private double mC2;
    private double mF1;
    private double mSalp0;
    private double mCalp0;
    private double mK2;
    private double mSalp1;
    private double mCalp1;
    private double mSsig1;
    private double mCsig1;
    private double mDn1;
    private double mStau1;
    private double mCtau1;
    private double mSomg1;
    private double mComg1;
    private double mA1m1;
    private double mA2m1;
    private double mA3c;
    private double mB11;
    private double mB21;
    private double mB31;
    private double mA4;
    private double mB41;

    private double mA13;
    private double mS13;

    // index zero elements of mC1a, mC1pa, mC2a, mC3a are unused
    // all the elements of mC4a are used.
    private double[] mC1a;
    private double[] mC1pa;
    private double[] mC2a;
    private double[] mC3a;
    private double[] mC4a;

    private int mCaps;

    /**
     * Constructor for a geodesic line staring at latitude <i>lat1</i>, longitude <i>lon1</i>,
     * and azimuth <i>azi1</i> (all in degrees).
     * If the point is at a pole, the azimuth is defined by keeping <i>lon1</i> fixed, writing
     * <i>lat1</i> = &plusmn;(90&deg; &minus; &epsilon;), and taking the limit &epsilon;
     * &rarr; 0+.
     *
     * @param g    a {@link Geodesic} object used to compute the necessary information about the
     *             GeodesicLine
     * @param lat1 latitude of point 1 (degrees). <i>lat1</i> should be in the range [&minus;90&deg;, 90&deg;].
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     */
    public GeodesicLine(
            final Geodesic g, final double lat1, final double lon1, final double azi1) {
        this(g, lat1, lon1, azi1, GeodesicMask.ALL);
    }

    /**
     * Constructor for a geodesic line starting at latitude <i>lat1</i>, longitude <i>lon1</i>,
     * and azimuth <i>azi1</i> (all in degrees) with a subset of the capabilities included.
     * The {@link GeodesicMask} values are:
     * <ul>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#LATITUDE} for the latitude <i>lat2</i>; this is
     *         added automatically
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#LONGITUDE} for the longitude <i>lon2</i>
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#AZIMUTH} for the azimuth <i>azi2</i>; this is
     *         added automatically
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#DISTANCE} for the distance <i>s12</i>
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#REDUCED_LENGTH} for the reduced length <i>m12</i>
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#GEODESIC_SCALE} for the geodesic scales <i>M12</i>
     *         and <i>M21</i>
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#AREA} for the area <i>S12</i>
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#DISTANCE_IN} permits the length of the geodesic
     *         to be given in terms of <i>s12</i>; without this capability the length can only be
     *         specified in terms of arc length
     *     </li>
     *     <li>
     *         <i>caps</i> |= {@link GeodesicMask#ALL} for all of the above.
     *     </li>
     * </ul>
     *
     * @param g    a {@link Geodesic} object used to compute the necessary information about the
     *             GeodesicLine.
     * @param lat1 latitude of point 1 (degrees).
     * @param lon1 longitude of point 1 (degrees).
     * @param azi1 azimuth at point 1 (degrees).
     * @param caps bitor'ed combination of {@link GeodesicMask} values specifying the capabilities
     *             the GeodesicLine object should possess, i.e., which quantities can be returned in
     *             calls to {@link #position}.
     */
    public GeodesicLine(final Geodesic g, final double lat1, final double lon1, double azi1,
                        final int caps) {
        azi1 = GeoMath.angNormalize(azi1);

        final double salp1;
        final double calp1;

        final Pair p = GeoMath.sincosd(GeoMath.angRound(azi1));
        salp1 = p.getFirst();
        calp1 = p.getSecond();

        lineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
    }

    protected GeodesicLine(
            final Geodesic g, final double lat1, final double lon1, final double azi1,
            final double salp1, final double calp1, final int caps, final boolean arcmode,
            final double s13A13) {
        lineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
        genSetDistance(arcmode, s13A13);
    }

    /**
     * Compute the position of point 2 which is a distance <i>s12</i> (meters) from point 1.
     * The values of <i>lon2</i> and <i>azi2</i> returned are in the range [&minus;180&deg;, 180&deg;].
     * The GeodesicLine object <i>must</i> have been constructed with <i>caps</i>
     * |= {@link GeodesicMask#DISTANCE_IN}; otherwise no parameters are set.
     *
     * @param s12 distance from point 1 to point 2 (meters); it can be negative.
     * @return a {@link GeodesicData} object with the following fields: <i>lat1</i>, <i>lon1</i>,
     * <i>azi1</i>, <i>lat2</i>, <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>. Some of these
     * results may be missing if the GeodesicLine did not include the relevant capability.
     */
    public GeodesicData position(final double s12) {
        return position(false, s12, GeodesicMask.STANDARD);
    }

    /**
     * Compute the position of point 2 which is a distance <i>s12</i> (meters) from point 1 and
     * with a subset of the geodesic results returned.
     * The GeodesicLine object <i>must</i> have been constructed with <i>caps</i> |=
     * {@link GeodesicMask#DISTANCE_IN}; otherwise no parameters are set.
     * Requesting a value which the GeodesicLine object is not capable of computing is not an error
     * (no parameters will be set). The value of <i>lon2</i> returned is normally in the range
     * [&minus;180&deg;, 180&deg;]; however if the <i>outmask</i> includes the
     * {@link GeodesicMask#LONG_UNROLL} flag, the longitude is "unrolled" so that the quantity
     * <i>lon2</i> &minus; <i>lon1</i> indicates how many times and in what sense the geodesic
     * encircles the ellipsoid.
     *
     * @param s12     distance from point 1 to point 2 (meters); it can be negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results
     *                should be returned.
     * @return a {@link GeodesicData} object including the requested results.
     */
    public GeodesicData position(final double s12, final int outmask) {
        return position(false, s12, outmask);
    }

    /**
     * Compute the position of point 2 which is an arc length <i>a12</i> (degrees) from point 1.
     * The values of <i>lon2</i> and <i>azi2</i> returned are in the range [&minus;180&deg;, 180&deg;].
     * The GeodesicLine object <i>must</i> have been constructed with <i>caps</i> |=
     * {@link GeodesicMask#DISTANCE_IN}; otherwise no parameters are set.
     *
     * @param a12 arc length from point 1 to point 2 (degrees); it can be negative.
     * @return a {@link GeodesicData} object with the following fields: <i>lat1</i>, <i>lon1</i>,
     * <i>azi1</i>, <i>lat2</i>, <i>lon2</i>, <i>azi2</i>, <i>s12</i>, <i>a12</i>. Some of these
     * results may be missing if the GeodesicLine did not include the relevant capability.
     */
    public GeodesicData arcPosition(final double a12) {
        return position(true, a12, GeodesicMask.STANDARD);
    }

    /**
     * Compute the position of point 2 which is an arc length <i>a12</i> (degrees) from point 1 and
     * with a subset of th geodesic results returned.
     * Requesting a value which the GeodesicLine object is not capable of computing is not an error
     * (no parameters will be set). The value of <i>lon2</i> returned is in the range
     * [&minus;180&deg;, 180&deg;], unless the <i>outmask</i> includes the
     * {@link GeodesicMask#LONG_UNROLL} flags.
     *
     * @param a12     arc length from point 1 to point 2 (degrees); it can be negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results
     *                should be returned.
     * @return a {@link GeodesicData} object giving <i>lat1</i>, <i>lon2</i>, <i>azi2</i>, and
     * <i>a12</i>.
     */
    public GeodesicData arcPosition(final double a12, final int outmask) {
        return position(true, a12, outmask);
    }

    /**
     * The general position function. {@link #position(double, int)} and {@link #arcPosition(double, int)}
     * are defined in terms of this function.
     * The {@link GeodesicMask} values possible for <i>outmask</i> are
     * <ul>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LATITUDE} for the latitude <i>lat2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LONGITUDE} for the latitude <i>lon2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#AZIMUTH} for the latitude <i>azi2</i>;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#DISTANCE} for the distance <i>s12</i>;
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
     *         <i>outmask</i> |= {@link GeodesicMask#ALL} for all of the above;
     *     </li>
     *     <li>
     *         <i>outmask</i> |= {@link GeodesicMask#LONG_UNROLL} to unroll <i>lon2</i> (instead of
     *         reducing it to the range [&minus;180&deg;, 180&deg;]).
     *     </li>
     * </ul>
     *
     * @param arcmode boolean flag determining the meaning of the second parameter; if arcmode is false,
     *                then the GeodesicLine object must have been constructed with <i>caps</i> |=
     *                {@link GeodesicMask#DISTANCE_IN}.
     * @param s12A12  if <i>arcmode</i> is false, this is the distance between point 1 and point 2 (meters);
     *                otherwise it is the arc length between point 1 and point 2 (degrees); it can be
     *                negative.
     * @param outmask a bitor'ed combination of {@link GeodesicMask} values specifying which results
     *                should be returned.
     * @return a {@link GeodesicData} object with the requested results. Requesting a value which the
     * GeodesicLine object is not capable of computing is not an error; Double.NaN is returned instead.
     */
    public GeodesicData position(final boolean arcmode, final double s12A12, int outmask) {
        outmask &= mCaps & GeodesicMask.OUT_MASK;
        final GeodesicData r = new GeodesicData();
        if (!(init() && (arcmode || (mCaps & (GeodesicMask.OUT_MASK & GeodesicMask.DISTANCE_IN)) != 0))) {
            // uninitialized or impossible distance calculation requested
            return r;
        }

        r.setLat1(mLat1);
        r.setAzi1(mAzi1);
        r.setLon1(((outmask & GeodesicMask.LONG_UNROLL) != 0) ? mLon1 : GeoMath.angNormalize(mLon1));

        // avoid warning about uninitialized b12
        double sig12;
        double ssig12;
        double csig12;
        double b12 = 0.0;
        double ab1 = 0.0;
        if (arcmode) {
            // interpret s12A12 as spherical arc length
            r.setA12(s12A12);
            sig12 = Math.toRadians(s12A12);

            final Pair p = GeoMath.sincosd(s12A12);
            ssig12 = p.getFirst();
            csig12 = p.getSecond();
        } else {
            // interpret s12A12 as distance
            r.setS12(s12A12);

            final double tau12 = s12A12 / (mB * (1 + mA1m1));
            final double s = Math.sin(tau12);
            final double c = Math.cos(tau12);

            // tau2 = tau1 + tau12
            b12 = -Geodesic.sinCosSeries(true, mStau1 * c + mCtau1 * s,
                    mCtau1 * c - mStau1 * s, mC1pa);
            sig12 = tau12 - (b12 - mB11);
            ssig12 = Math.sin(sig12);
            csig12 = Math.cos(sig12);

            if (Math.abs(mF) > 0.01) {
                // reverted distance series is inaccurate for |f| > 1/100, so correct sig12 with 1
                // Newton iteration. The following table shows the approximate maximum error for
                // a = WGSa() and various f relative to GeodesicExact.
                // erri = the error in the inverse solution (nm)
                // errd = the error in the direct solution (series only) (nm)
                // errda = the error in the direct solution (series + 1 Newton) (nm)

                //  f       erri        errd        errda
                //  -1/5    12e6        1.2e9       69e6
                //  -1/10   123e3       12e6        765e3
                //  -1/20   1110        108e3       7155
                //  -1/50   18.63       200.9       27.12
                //  -1/100  18.63       23.78       23.37
                //  -1/150  18.63       21.05       20.26
                //  1/150   22.35       24.73       25.83
                //  1/100   22.35       25.03       25.31
                //  1/50    29.80       231.9       30.44
                //  1/20    5376        146e3       10e3
                //  1/10    829e3       22e6        1.5e6
                //  1/5     157e6       3.8e9       280e6

                final double ssig2 = mSsig1 * csig12 + mCsig1 * ssig12;
                final double csig2 = mCsig1 * csig12 - mSsig1 * ssig12;
                b12 = Geodesic.sinCosSeries(true, ssig2, csig2, mC1a);

                final double serr = (1 + mA1m1) * (sig12 + (b12 - mB11)) - s12A12 / mB;
                sig12 = sig12 - serr / Math.sqrt(1 + mK2 * GeoMath.sq(ssig2));
                ssig12 = Math.sin(sig12);
                csig12 = Math.cos(sig12);
                // update b12 below
            }
            r.setA12(Math.toDegrees(sig12));
        }

        final double ssig2 = mSsig1 * csig12 + mCsig1 * ssig12;
        double csig2 = mCsig1 * csig12 - mSsig1 * ssig12;
        final double sbet2;
        double cbet2;
        final double salp2;
        final double calp2;
        // sig2 = sig1 + sig12


        final double dn2 = Math.sqrt(1 + mK2 * GeoMath.sq(ssig2));
        if ((outmask & (GeodesicMask.DISTANCE | GeodesicMask.REDUCED_LENGTH |
                GeodesicMask.GEODESIC_SCALE)) != 0) {
            if (arcmode || Math.abs(mF) > 0.01) {
                b12 = Geodesic.sinCosSeries(true, ssig2, csig2, mC1a);
            }
            ab1 = (1 + mA1m1) * (b12 - mB11);
        }

        // sin(bet2) = cos(alp0) * sin(sig2)
        sbet2 = mCalp0 * ssig2;
        // alt: cbet2 = hypot(csig2, salp0 * ssig2)
        cbet2 = GeoMath.hypot(mSalp0, mCalp0 * csig2);
        if (cbet2 == 0) {
            // i.e., salp0 = 0, csig2 = 0. Break the degeneracy in this case
            cbet2 = csig2 = Geodesic.TINY;
        }

        // tan(alp0) = cos(sig2) * tan(alp2)
        salp2 = mSalp0;
        // no need to normalize
        calp2 = mCalp0 * csig2;

        if ((outmask & GeodesicMask.DISTANCE) != 0 && arcmode) {
            r.setS12(mB * ((1 + mA1m1) * sig12 + ab1));
        }

        if ((outmask & GeodesicMask.LONGITUDE) != 0) {
            // tan(omg2) = sin(alp0) * tan(sig2)

            // no need to normalize east or west going?
            //noinspection all
            final double somg2 = mSalp0 * ssig2;
            final double e = GeoMath.copysign(1, mSalp0);

            // omg12 = omg2 - omg1
            final double omg12 = ((outmask & GeodesicMask.LONG_UNROLL) != 0) ?
                    e * (sig12 - (Math.atan2(ssig2, csig2) - Math.atan2(mSsig1, mCsig1)) +
                            (Math.atan2(e * somg2, csig2) - Math.atan2(e * mSomg1, mComg1))) :
                    Math.atan2(somg2 * mComg1 - csig2 * mSomg1,
                            csig2 * mComg1 + somg2 * mSomg1);

            final double lam12 = omg12 + mA3c * (sig12 +
                    (Geodesic.sinCosSeries(true, ssig2, csig2, mC3a) - mB31));
            final double lon12 = Math.toDegrees(lam12);
            r.setLon2(((outmask & GeodesicMask.LONG_UNROLL) != 0) ? mLon1 + lon12 :
                    GeoMath.angNormalize(r.getLon1() + GeoMath.angNormalize(lon12)));
        }

        if ((outmask & GeodesicMask.LATITUDE) != 0) {
            r.setLat2(GeoMath.atan2d(sbet2, mF1 * cbet2));
        }

        if ((outmask & GeodesicMask.AZIMUTH) != 0) {
            r.setAzi2(GeoMath.atan2d(salp2, calp2));
        }

        if ((outmask & (GeodesicMask.REDUCED_LENGTH | GeodesicMask.GEODESIC_SCALE)) != 0) {
            final double b22 = Geodesic.sinCosSeries(true, ssig2, csig2, mC2a);
            final double ab2 = (1 + mA2m1) * (b22 - mB21);
            final double j12 = (mA1m1 - mA2m1) * sig12 + (ab1 - ab2);
            if ((outmask & GeodesicMask.REDUCED_LENGTH) != 0) {
                // add parens around (mCsig1 * ssig2) and (mSsig1 * csig2) to ensure
                // accurate cancellation in the case of coincident points
                r.setM12(mB * ((dn2 * (mCsig1 * ssig2) - mDn1 * (mSsig1 * csig2)) -
                        mCsig1 * csig2 * j12));
            }
            if ((outmask & GeodesicMask.GEODESIC_SCALE) != 0) {
                final double t = mK2 * (ssig2 - mSsig1) * (ssig2 + mSsig1) / (mDn1 + dn2);
                r.setScaleM12(csig12 + (t * ssig2 - csig2 * j12) * mSsig1 / mDn1);
                r.setScaleM21(csig12 - (t * mSsig1 - mCsig1 * j12) * ssig2 / dn2);
            }
        }

        if ((outmask & GeodesicMask.AREA) != 0) {
            final double b42 = Geodesic.sinCosSeries(false, ssig2, csig2, mC4a);
            final double salp12;
            final double calp12;
            if (mCalp0 == 0 || mSalp0 == 0) {
                //alp12 = alp2 - alp1, used in atan2 so no need to normalize
                salp12 = salp2 * mCalp1 - calp2 * mSalp1;
                calp12 = calp2 * mCalp1 + salp2 * mSalp1;
            } else {
                // tan(alp) = tan(alp0) * sec(sig)
                // tan(alp2-alp1) = (tan(alp2) -tan(alp1)) / (tan(alp2)*tan(alp1)+1)
                // = calp0 * salp0 * (csig1-csig2) / (salp0^2 + calp0^2 * csig1*csig2)
                // if csig12 > 0, write
                //   csig1 - csig2 = ssig12 * (csig1 * ssig12 / (1 + csig12) + ssig1)
                // else
                //   csig1 - csig2 = csig1 * (1 - csig12) + ssig12 * ssig1
                // no need to normalize
                salp12 = mCalp0 * mSalp0 *
                        (csig12 <= 0 ? mCsig1 * (1 - csig12) + ssig12 * mSsig1 :
                                ssig12 * (mCsig1 * ssig12 / (1 + csig12) + mSsig1));
                calp12 = GeoMath.sq(mSalp0) + GeoMath.sq(mCalp0) * mCsig1 * csig2;
            }
            r.setAreaS12(mC2 * Math.atan2(salp12, calp12) + mA4 * (b42 - mB41));
        }

        return r;
    }

    /**
     * Specify position of point 3 in terms of distance.
     * This is only useful if the GeodesicLine object has been constructed with <i>caps</i> |=
     * {@link GeodesicMask#DISTANCE_IN}.
     *
     * @param s13 the distance from point 1 to point 3 (meters); it can be negative.
     */
    public void setDistance(final double s13) {
        mS13 = s13;
        final GeodesicData g = position(false, mS13, 0);
        mA13 = g.getA12();
    }

    /**
     * Specify position of point 3 in terms of either distance or arc length.
     *
     * @param arcmode boolean flag determining the meaning of the second parameter; if <i>arcmode</i>
     *                is false, then the GeodesicLine object must have been constructed with
     *                <i>caps</i> |= {@link GeodesicMask#DISTANCE_IN}.
     * @param s13A13  if <i>arcmode</i> is false, this is the distance from point 1 to point 3 (meters);
     *                otherwise it is the arc length from point 1 to point 3 (degrees); it can be
     *                negative.
     */
    public void genSetDistance(final boolean arcmode, final double s13A13) {
        if (arcmode) {
            setArc(s13A13);
        } else {
            setDistance(s13A13);
        }
    }

    /**
     * Gets the latitude of point 1 (degrees).
     *
     * @return <i>lat1</i> the latitude of point 1 (degrees).
     */
    public double getLatitude() {
        return init() ? mLat1 : Double.NaN;
    }

    /**
     * Gets the longitude of point 1 (degrees).
     *
     * @return <i>lon1</i> the longitude of point 1 (degrees).
     */
    public double getLongitude() {
        return init() ? mLon1 : Double.NaN;
    }

    /**
     * Gets the azimuth (degrees) of the geodesic line at point 1.
     *
     * @return <i>azi1</i> the azimuth (degrees) of the geodesic line at point 1.
     */
    public double getAzimuth() {
        return init() ? mAzi1 : Double.NaN;
    }

    /**
     * Gets a pair of sine and cosine of <i>azi1</i> the azimuth (degrees) of the geodesic line at
     * point 1.
     *
     * @return pair of sine and cosine of <i>azi1</i> the azimuth (degrees) of the geodesic line
     * at point 1.
     */
    public Pair getAzimuthCosines() {
        return new Pair(init() ? mSalp1 : Double.NaN, init() ? mCalp1 : Double.NaN);
    }

    /**
     * Gets the azimuth (degrees) of the geodesic line as it crosses the equator in a northward
     * direction.
     *
     * @return <i>azi0</i> the azimuth (degrees) of the geodesic line as it crosses the equator in a
     * northward direction.
     */
    public double getEquatorialAzimuth() {
        return init() ? GeoMath.atan2d(mSalp0, mCalp0) : Double.NaN;
    }

    /**
     * Gets a pair of sine and cosine of <i>azi0</i> the azimuth of the godesic line as it crosses the
     * equator in a northward direction.
     *
     * @return pair of sine and cosine of <i>azi0</i> the azimuth of the godesic line as it crosses
     * the equator in a northward direction.
     */
    public Pair getEquatorialAzimuthCosines() {
        return new Pair(init() ? mSalp0 : Double.NaN, init() ? mCalp0 : Double.NaN);
    }

    /**
     * Gets the arc length (degrees) between the northward equatorial crossing and point 1.
     *
     * @return <i>a1</i> the arc length (degrees) between the northward equatorial crossing and
     * point 1.
     */
    public double getEquatorialArc() {
        return init() ? GeoMath.atan2d(mSsig1, mCsig1) : Double.NaN;
    }

    /**
     * Gets the equatorial radius of the ellipsoid (meters). This is the value inherited from the
     * Geodesic object used in the constructor.
     *
     * @return <i>a</i> the equatorial radius of the ellipsoid (meters).
     */
    public double getMajorRadius() {
        return init() ? mA : Double.NaN;
    }

    /**
     * Gets the flattening of the ellipsoid. This is the value inherited from the Geodesic object
     * used in the constructor.
     *
     * @return <i>f</i> the flattening of the ellipsoid.
     */
    public double getFlattening() {
        return init() ? mF : Double.NaN;
    }

    /**
     * Gets the computational capabilities that this object was constructed with. LATITUDE and AZIMUTH
     * are always included.
     *
     * @return <i>caps</i> the computation capabilities that this object was constructed with.
     */
    public int getCapabilities() {
        return mCaps;
    }

    /**
     * Indicates whether this GeodesicLine object has all tested capabilities
     *
     * @param testcaps a set of bitor'ed {@link GeodesicMask} values.
     * @return true if the GeodesicLine object has all these capabilities.
     */
    public boolean capabilities(int testcaps) {
        testcaps &= GeodesicMask.OUT_ALL;
        return (mCaps & testcaps) == testcaps;
    }

    /**
     * The distance or arc length to point 3.
     *
     * @param arcmode boolean flag determining the meaning of returned value.
     * @return <i>s13</i> if <i>arcmode</i> is false; <i>a13</i> if <i>arcmode</i> is true.
     */
    public double genDistance(final boolean arcmode) {
        double tmp = arcmode ? mA13 : mS13;
        return init() ? tmp : Double.NaN;
    }

    /**
     * Gets the distance to point 3 (meters).
     *
     * @return <i>s13</i> the disance to point 3 (meters).
     */
    public double getDistance() {
        return genDistance(false);
    }

    /**
     * Gets the arc length to point 3 (degrees).
     *
     * @return <i>a13</i> the arc length to point 3 (degrees).
     */
    public double getArc() {
        return genDistance(true);
    }

    /**
     * Specify position of point 3 in terms of arc length.
     * The distance <i>s13</i> is only set if the GeodesicLine object has been constructed with
     * <i>caps</i> |= {@link GeodesicMask#DISTANCE}.
     *
     * @param a13 the arc length from point 1 to point 3 (degrees); it can be negative.
     */
    void setArc(final double a13) {
        mA13 = a13;
        GeodesicData g = position(true, mA13, GeodesicMask.DISTANCE);
        mS13 = g.getS12();
    }

    /**
     * @return true if the object has been initialized.
     */
    private boolean init() {
        return mCaps != 0;
    }

    private void lineInit(
            final Geodesic g, final double lat1, final double lon1, final double azi1,
            final double salp1, final double calp1, final int caps) {
        mA = g.mA;
        mF = g.mF;
        mB = g.mB;
        mC2 = g.mC2;
        mF1 = g.mF1;

        // always allow latitude and azimuth and unrolling the longitude
        mCaps = caps | GeodesicMask.LATITUDE | GeodesicMask.AZIMUTH |
                GeodesicMask.LONG_UNROLL;

        mLat1 = GeoMath.latFix(lat1);
        mLon1 = lon1;
        mAzi1 = azi1;
        mSalp1 = salp1;
        mCalp1 = calp1;

        double cbet1;
        double sbet1;

        Pair p = GeoMath.sincosd(GeoMath.angRound(mLat1));
        sbet1 = mF1 * p.getFirst();
        cbet1 = p.getSecond();

        // ensure cbet1 = +epsilon at poles
        p = GeoMath.norm(sbet1, cbet1);
        sbet1 = p.getFirst();
        cbet1 = Math.max(Geodesic.TINY, p.getSecond());

        mDn1 = Math.sqrt(1 + g.mEp2 * GeoMath.sq(sbet1));

        // evaluate alp0 from sin(alp1) * cos(bet1) = sin(alp0),

        // alp0 in [0, pi/2 - |bet1|]
        mSalp0 = mSalp1 * cbet1;

        // alt: calp0 = hypot(sbet1, calp1 * cbet1). The following is slightly
        // better (consider the case salp1 = 0).
        mCalp0 = GeoMath.hypot(mCalp1, mSalp1 * sbet1);

        // Evaluate sig with tan(bet1) = tan(sig1) * cos(alp1).
        // sig = 0 is nearest northward crossing of the equator.
        // With bet1 = 0, alp1 = pi/2, we have sig1 = 0 (equatorial line).
        // With bet1 = pi/2, alp1 = -pi, sig1 = pi/2
        // With bet1 = -pi/2, alp1 = 0, sig1 = -pi/2
        // Evaluate omg1 with tan(omg1) = sin(alp0) * tan(sig1).
        // With alp0 in (0, pi/2], quadrants for sig and omg coincide.
        // No atan2(0,0) ambiguity at poles since cbet1 = +epsilon
        // With alp0 = 0, omg1 = 0 for alp1 = 0, omg1 = pi for alp1 = pi.
        mSsig1 = sbet1;
        mSomg1 = mSalp0 * sbet1;
        mCsig1 = mComg1 = sbet1 != 0 || mCalp1 != 0 ? cbet1 * mCalp1 : 1;

        p = GeoMath.norm(mSsig1, mCsig1);
        mSsig1 = p.getFirst();
        // sig 1 in (-pi, pi]
        mCsig1 = p.getSecond();

        // GeoMath.norm(mSomg1, mComg1); -- don't need to normalize!

        mK2 = GeoMath.sq(mCalp0) * g.mEp2;
        final double eps = mK2 / (2 * (1 + Math.sqrt(1 + mK2)) + mK2);

        if ((mCaps & GeodesicMask.CAP_C1) != 0) {
            mA1m1 = Geodesic.a1m1f(eps);
            mC1a = new double[NC1 + 1];
            Geodesic.c1f(eps, mC1a);
            mB11 = Geodesic.sinCosSeries(true, mSsig1, mCsig1, mC1a);
            final double s = Math.sin(mB11);
            final double c = Math.cos(mB11);
            // tau1 = sig1 + b11
            mStau1 = mSsig1 * c + mCsig1 * s;
            mCtau1 = mCsig1 * c - mSsig1 * s;
            // not necessary because c1pa rverts c1a
            // mB11 = -sinCosSeries(true, mStau1, mCtau1, mC1pa, NC1P)
        }

        if ((mCaps & GeodesicMask.CAP_C1P) != 0) {
            mC1pa = new double[NC1P + 1];
            Geodesic.c1pf(eps, mC1pa);
        }

        if ((mCaps & GeodesicMask.CAP_C2) != 0) {
            mC2a = new double[NC2 + 1];
            mA2m1 = Geodesic.a2m1f(eps);
            Geodesic.c2f(eps, mC2a);
            mB21 = Geodesic.sinCosSeries(true, mSsig1, mCsig1, mC2a);
        }

        if ((mCaps & GeodesicMask.CAP_C3) != 0) {
            mC3a = new double[NC3];
            g.c3f(eps, mC3a);
            mA3c = -mF * mSalp0 * g.a3f(eps);
            mB31 = Geodesic.sinCosSeries(true, mSsig1, mCsig1, mC3a);
        }

        if ((mCaps & GeodesicMask.CAP_C4) != 0) {
            mC4a = new double[NC4];
            g.c4f(eps, mC4a);
            // multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0)
            mA4 = GeoMath.sq(mA) * mCalp0 * mSalp0 * g.mE2;
            mB41 = Geodesic.sinCosSeries(false, mSsig1, mCsig1, mC4a);
        }
    }
}
