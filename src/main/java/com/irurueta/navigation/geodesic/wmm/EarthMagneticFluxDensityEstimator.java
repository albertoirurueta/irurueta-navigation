/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.geodesic.wmm;

import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;

/**
 * Estimates Earth magnetic flux density resolved around NED frame at a
 * given Earth location.
 * Magnetic flux density depends of magnitude of the magnetic field,
 * the declination and the dip angle.
 * Magnitude typically varies from about 30 µT at the equator to about 60 µT
 * at the poles.
 * The dip (or inclination) angle is essentially the magnetic latitude and
 * it is typically within about 10º of the geodetic latitude.
 * The declination angle gives the bearing of the magnetic field from
 * true north, and is the only of the three parameters needed to determine a
 * user's heading form magnetic field measurements.
 * The declination angle may be calculated as a function of position and
 * time using global models, such as the 275-coefficient International
 * Geomagnetic Reference Field (IGRF) or the 336-coefficient U.D/U.K
 * World Magnetic Model (WMM).
 * <p>
 * IGRF: https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
 * WMM: https://www.ngdc.noaa.gov/geomag/WMM
 */
public class EarthMagneticFluxDensityEstimator {

    /**
     * Private constructor to prevent instantiation.
     */
    private EarthMagneticFluxDensityEstimator() { }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param magnitude   magnitude of magnetic field expressed in Teslas (T).
     * @param declination declination angle expressed in radians.
     * @param dip         dip (or inclination) angle expressed in radians.
     * @param result      instance where magnetic flux will be stored resolved
     *                    around NED frame.
     */
    public static void estimate(
            final double magnitude,
            final double declination,
            final double dip,
            final NEDMagneticFluxDensity result) {

        final double cosDeclination = Math.cos(declination);
        final double sinDeclination = Math.sin(declination);

        final double cosDip = Math.cos(dip);
        final double sinDip = Math.sin(dip);

        final double bn = cosDeclination * cosDip * magnitude;
        final double be = sinDeclination * cosDip * magnitude;
        final double bd = sinDip * magnitude;

        result.setCoordinates(bn, be, bd);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param magnitude   magnitude of magnetic field expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (or inclination) angle.
     * @param result      instance where magnetic flux will be stored resolved
     *                    around NED frame.
     */
    public static void estimate(
            final double magnitude,
            final Angle declination,
            final Angle dip,
            final NEDMagneticFluxDensity result) {
        estimate(magnitude, convertAngle(declination), convertAngle(dip),
                result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param magnitude   magnitude of magnetic field expressed in Teslas (T).
     * @param declination declination angle expressed in radians.
     * @param dip         dip (or inclination) angle expressed in radians.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public static NEDMagneticFluxDensity estimate(
            final double magnitude,
            final double declination,
            final double dip) {
        final NEDMagneticFluxDensity result =
                new NEDMagneticFluxDensity();
        estimate(magnitude, declination, dip, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param magnitude   magnitude of magnetic field expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (or inclination) angle.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public static NEDMagneticFluxDensity estimate(
            final double magnitude,
            final Angle declination,
            final Angle dip) {
        return estimate(magnitude, convertAngle(declination),
                convertAngle(dip));
    }

    /**
     * Gets declination angle expressed in radians.
     *
     * @param b a magnetic flux density resolved around NED frame.
     * @return declination angle.
     */
    public static double getDeclination(final NEDMagneticFluxDensity b) {
        final double bn = b.getBn();
        final double be = b.getBe();

        return Math.atan2(be, bn);
    }

    /**
     * Gets declination angle.
     *
     * @param b      a magnetic flux density resolved around NED frame.
     * @param result instance where declination angle will be stored.
     */
    public static void getDeclinationAsAngle(
            final NEDMagneticFluxDensity b, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(b));
    }

    /**
     * Gets declination angle.
     *
     * @param b a magnetic flux density resolved around NED frame.
     * @return declination angle.
     */
    public static Angle getDeclinationAsAngle(
            final NEDMagneticFluxDensity b) {
        return new Angle(getDeclination(b), AngleUnit.RADIANS);
    }

    /**
     * Gets dip (a.k.a. inclination) angle expressed in radians.
     *
     * @param b a magnetic flux density resolved around NED frame.
     * @return dip angle.
     */
    public static double getDip(final NEDMagneticFluxDensity b) {
        final double bn = b.getBn();
        final double be = b.getBe();
        final double bd = b.getBd();

        final double bn2 = bn * bn;
        final double be2 = be * be;
        return Math.atan(bd / Math.sqrt(bn2 + be2));
    }

    /**
     * Gets dip (a.k.a. inclination) angle expressed in radians.
     *
     * @param b      a magnetic flux density resolved around NED frame.
     * @param result instance where dip angle will be stored.
     */
    public static void getDipAsAngle(
            final NEDMagneticFluxDensity b, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(b));
    }

    /**
     * Gets dip (a.k.a. inclination) angle expressed in radians.
     *
     * @param b a magnetic flux density resolved around NED frame.
     * @return dip angle.
     */
    public static Angle getDipAsAngle(final NEDMagneticFluxDensity b) {
        return new Angle(getDip(b), AngleUnit.RADIANS);
    }

    /**
     * Converts an angle instanto into radians.
     *
     * @param angle angle to be converted.
     * @return converted value  into radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }
}
