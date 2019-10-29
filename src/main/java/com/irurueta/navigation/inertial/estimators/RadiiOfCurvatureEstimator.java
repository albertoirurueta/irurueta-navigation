package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;

/**
 * Calculates radii of curvature at a given latitude.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/InertialDemoECEF
 */
@SuppressWarnings("WeakerAccess")
public class RadiiOfCurvatureEstimator {
    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude expressed in radians (rad).
     * @param result instance where estimated radii of curvature will be stored.
     */
    public void estimate(final double latitude, final RadiiOfCurvature result) {
        estimateRadiiOfCurvature(latitude, result);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude expressed in radians (rad).
     * @return a new radii of curvature instance containing meridian and transverse
     * radii of curvature.
     */
    public RadiiOfCurvature estimateAndReturnNew(final double latitude) {
        return estimateRadiiOfCurvatureAndReturnNew(latitude);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude.
     * @param result instance where estimated radii of curvature will be stored.
     */
    public void estimate(final Angle latitude, final RadiiOfCurvature result) {
        estimateRadiiOfCurvature(latitude, result);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude.
     * @return a new radii of curvature instance containing meridian and transverse
     * radii of curvature.
     */
    public RadiiOfCurvature estimateAndReturnNew(final Angle latitude) {
        return estimateRadiiOfCurvatureAndReturnNew(latitude);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude expressed in radians (rad).
     * @param result instance where estimated radii of curvature will be stored.
     */
    public static void estimateRadiiOfCurvature(final double latitude,
                                                final RadiiOfCurvature result) {
        final double tmp = 1.0 - Math.pow(EARTH_ECCENTRICITY * Math.sin(latitude), 2.0);

        // Calculate meridian radius of curvature using (2.105)
        final double rn = EARTH_EQUATORIAL_RADIUS_WGS84
                * (1.0 - Math.pow(EARTH_ECCENTRICITY, 2.0)) / Math.pow(tmp, 1.5);

        // Calculate transverse radius of curvature using (2.106)
        final double re = EARTH_EQUATORIAL_RADIUS_WGS84 / Math.sqrt(tmp);

        result.setValues(rn, re);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude expressed in radians (rad).
     * @return a new radii of curvature instance containing meridian and transverse
     * radii of curvature.
     */
    public static RadiiOfCurvature estimateRadiiOfCurvatureAndReturnNew(final double latitude) {
        final RadiiOfCurvature result = new RadiiOfCurvature();
        estimateRadiiOfCurvature(latitude, result);
        return result;
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude.
     * @param result instance where estimated radii of curvature will be stored.
     */
    public static void estimateRadiiOfCurvature(final Angle latitude,
                                                final RadiiOfCurvature result) {
        estimateRadiiOfCurvature(AngleConverter.convert(
                latitude.getValue().doubleValue(), latitude.getUnit(), AngleUnit.RADIANS),
                result);
    }

    /**
     * Estimates radii of curvature (meridian and transverse radii of curvature) at
     * provided geodetic latitude.
     *
     * @param latitude geodetic latitude.
     * @return a new radii of curvature instance containing meridian and transverse
     * radii of curvature.
     */
    public static RadiiOfCurvature estimateRadiiOfCurvatureAndReturnNew(final Angle latitude) {
        final RadiiOfCurvature result = new RadiiOfCurvature();
        estimateRadiiOfCurvature(latitude, result);
        return result;
    }
}
