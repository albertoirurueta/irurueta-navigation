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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;

/**
 * Estimates magnetic flux density resolved around body coordinates for
 * a given Earth magnetic flux density an a given body attitude.
 */
public class BodyMagneticFluxDensityEstimator {

    /**
     * Private constructor to prevent instantiation.
     */
    private BodyMagneticFluxDensityEstimator() { }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle expressed in radians (rad).
     * @param dip         dip (a.k.a. inclination) angle expressed in radians
     *                    (rad).
     * @param roll        body roll angle expressed in radians (rad).
     * @param pitch       body pitch angle expressed in radians (rad).
     * @param yaw         body yaw angle expressed in radians (rad).
     * @param result      instance where resulting magnetic flux density
     *                    measured in body coordinates will be stored.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void estimate(
            final double magnitude, final double declination, final double dip,
            final double roll, final double pitch, final double yaw,
            final BodyMagneticFluxDensity result) {

        final double magneticHeading = yaw - declination;

        final double cosHeading = Math.cos(magneticHeading);
        final double sinHeading = Math.sin(magneticHeading);

        final double cosDip = Math.cos(dip);
        final double sinDip = Math.sin(dip);

        // notice that bn and be are not really pointing towards north and
        // east, instead they are affected by the amount of declination.
        final double bn = cosHeading * cosDip * magnitude;
        final double be = sinHeading * cosDip * magnitude;
        final double bd = sinDip * magnitude;

        final double sinRoll = Math.sin(roll);
        final double cosRoll = Math.cos(roll);
        final double sinPitch = Math.sin(pitch);
        final double cosPitch = Math.cos(pitch);

        final double bx = cosPitch * bn - sinPitch * bd;
        final double by = sinRoll * sinPitch * bn - cosRoll * be
                + sinRoll * cosPitch * bd;
        final double bz = cosRoll * sinPitch * bn + sinRoll * be
                + cosRoll * cosPitch * bd;

        result.setCoordinates(bx, by, bz);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle expressed in radians (rad).
     * @param dip         dip (a.k.a. inclination) angle expressed in radians
     *                    (rad).
     * @param c           coordinate transformation from NED to body
     *                    coordinates.
     * @param result      instance where resulting magnetic flux density
     *                    measured in body coordinates will be stored.
     */
    public static void estimate(
            final double magnitude, final double declination, final double dip,
            final CoordinateTransformation c,
            final BodyMagneticFluxDensity result) {
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        estimate(magnitude, declination, dip, roll, pitch, yaw, result);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param roll   body roll angle expressed in radians (rad).
     * @param pitch  body pitch angle expressed in radians (rad).
     * @param yaw    body yaw angle expressed in radians (rad).
     * @param result instance where resulting magnetic flux density
     *               measured in body coordinates will be stored.
     */
    public static void estimate(
            final NEDMagneticFluxDensity earthB,
            final double roll, final double pitch, final double yaw,
            final BodyMagneticFluxDensity result) {

        final CoordinateTransformation c = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        estimate(earthB, c, result);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param c      coordinate transformation from NED to body coordinates.
     * @param result instance where resulting magnetic flux density measured
     *               in body coordinates will be stored.
     */
    public static void estimate(
            final NEDMagneticFluxDensity earthB,
            final CoordinateTransformation c,
            final BodyMagneticFluxDensity result) {

        try {
            final Matrix bm = earthB.asMatrix();
            final Matrix cbn = c.getMatrix();
            cbn.multiply(bm);

            // cbn now contains magnetic flux density in body coordinates
            final double bx = cbn.getElementAtIndex(0);
            final double by = cbn.getElementAtIndex(1);
            final double bz = cbn.getElementAtIndex(2);

            result.setCoordinates(bx, by, bz);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (a.k.a. inclination) angle.
     * @param roll        body roll angle expressed.
     * @param pitch       body pitch angle.
     * @param yaw         body yaw angle.
     * @param result      instance where resulting magnetic flux density
     *                    measured in body coordinates will be stored.
     */
    public static void estimate(
            final double magnitude, final Angle declination, final Angle dip,
            final Angle roll, final Angle pitch, final Angle yaw,
            final BodyMagneticFluxDensity result) {
        estimate(magnitude, convertAngle(declination), convertAngle(dip),
                convertAngle(roll), convertAngle(pitch), convertAngle(yaw),
                result);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (a.k.a. inclination) angle.
     * @param c           coordinate transformation from NED to body
     *                    coordinates.
     * @param result      instance where resulting magnetic flux density
     *                    measured in body coordinates will be stored.
     */
    public static void estimate(
            final double magnitude, final Angle declination, final Angle dip,
            final CoordinateTransformation c,
            final BodyMagneticFluxDensity result) {
        estimate(magnitude, convertAngle(declination), convertAngle(dip),
                c, result);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param roll   body roll angle.
     * @param pitch  body pitch angle.
     * @param yaw    body yaw angle.
     * @param result instance where resulting magnetic flux density
     *               measured in body coordinates will be stored.
     */
    public static void estimate(
            final NEDMagneticFluxDensity earthB,
            final Angle roll, final Angle pitch, final Angle yaw,
            final BodyMagneticFluxDensity result) {
        estimate(earthB, convertAngle(roll), convertAngle(pitch),
                convertAngle(yaw), result);
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle expressed in radians (rad).
     * @param dip         dip (a.k.a. inclination) angle expressed in radians
     *                    (rad).
     * @param roll        body roll angle expressed in radians (rad).
     * @param pitch       body pitch angle expressed in radians (rad).
     * @param yaw         body yaw angle expressed in radians (rad).
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final double magnitude, final double declination, final double dip,
            final double roll, final double pitch, final double yaw) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(magnitude, declination, dip, roll, pitch, yaw, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle expressed in radians (rad).
     * @param dip         dip (a.k.a. inclination) angle expressed in radians
     *                    (rad).
     * @param c           coordinate transformation from NED to body
     *                    coordinates.
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final double magnitude, final double declination, final double dip,
            final CoordinateTransformation c) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(magnitude, declination, dip, c, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param roll   body roll angle expressed in radians (rad).
     * @param pitch  body pitch angle expressed in radians (rad).
     * @param yaw    body yaw angle expressed in radians (rad).
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final NEDMagneticFluxDensity earthB,
            final double roll, final double pitch, final double yaw) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(earthB, roll, pitch, yaw, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param c      coordinate transformation from NED to body coordinates.
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final NEDMagneticFluxDensity earthB,
            final CoordinateTransformation c) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(earthB, c, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (a.k.a. inclination) angle.
     * @param roll        body roll angle expressed.
     * @param pitch       body pitch angle.
     * @param yaw         body yaw angle.
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final double magnitude, final Angle declination, final Angle dip,
            final Angle roll, final Angle pitch, final Angle yaw) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(magnitude, declination, dip, roll, pitch, yaw, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param magnitude   magnitude (a.k.a. intensity) of Earth magnetic flux
     *                    density expressed in Teslas (T).
     * @param declination declination angle.
     * @param dip         dip (a.k.a. inclination) angle.
     * @param c           coordinate transformation from NED to body
     *                    coordinates.
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final double magnitude, final Angle declination, final Angle dip,
            final CoordinateTransformation c) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(magnitude, declination, dip, c, result);
        return result;
    }

    /**
     * Computes expected measured body magnetic flux density for a given Earth
     * magnetic flux density and a certain body attitude (a.k.a. orientation).
     *
     * @param earthB Earth magnetic flux density.
     * @param roll   body roll angle.
     * @param pitch  body pitch angle.
     * @param yaw    body yaw angle.
     * @return measured magnetic flux density resolved in body coordinates.
     */
    public static BodyMagneticFluxDensity estimate(
            final NEDMagneticFluxDensity earthB,
            final Angle roll, final Angle pitch, final Angle yaw) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        estimate(earthB, roll, pitch, yaw, result);
        return result;
    }

    /**
     * Converts a given angle instance into radians.
     *
     * @param angle angle to be converted.
     * @return converted value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }
}
