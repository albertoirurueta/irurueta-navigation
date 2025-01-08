/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIorECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;

/**
 * Converts from ECEF frame to NED frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/ECEF_to_NED.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/ECEF_to_NED.m
 * </a>
 */
public class ECEFtoNEDFrameConverter implements FrameConverter<ECEFFrame, NEDFrame> {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Converts source ECEF frame to a new NED frame instance.
     *
     * @param source source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public NEDFrame convertAndReturnNew(final ECEFFrame source) {
        final var result = new NEDFrame();
        convert(source, result);
        return result;
    }

    /**
     * Converts source ECEF frame to destination NED frame.
     *
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    @Override
    public void convert(final ECEFFrame source, final NEDFrame destination) {
        convertECEFtoNED(source, destination);
    }

    /**
     * Gets source frame type.
     *
     * @return source frame type.
     */
    @Override
    public FrameType getSourceType() {
        return FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME;
    }

    /**
     * Gets destination frame type.
     *
     * @return destination frame type.
     */
    @Override
    public FrameType getDestinationType() {
        return FrameType.LOCAL_NAVIGATION_FRAME;
    }

    /**
     * Converts source ECEF frame to a new NED frame instance.
     *
     * @param source source frame to convert from.
     * @return a new destination frame instance.
     */
    public static NEDFrame convertECEFtoNEDAndReturnNew(final ECEFFrame source) {
        final var result = new NEDFrame();
        convertECEFtoNED(source, result);
        return result;
    }

    /**
     * Converts source ECEF frame to destination NED frame.
     *
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void convertECEFtoNED(final ECEFFrame source, final NEDFrame destination) {
        final var x = source.getX();
        final var y = source.getY();
        final var z = source.getZ();

        // Convert position using Borkowski closed-form exact solution from (2.113).
        final var longitude = Math.atan2(y, x);

        // From (C.29) and (C.30)
        final var ecc2 = EARTH_ECCENTRICITY * EARTH_ECCENTRICITY;
        final var k1 = Math.sqrt(1.0 - ecc2) * Math.abs(z);
        final var k2 = ecc2 * EARTH_EQUATORIAL_RADIUS_WGS84;

        final var x2 = x * x;
        final var y2 = y * y;
        final var beta = Math.sqrt(x2 + y2);

        final var e = (k1 - k2) / beta;
        final var f = (k1 + k2) / beta;

        // From (C.31)
        final var p = 4.0 / 3.0 * (e * f + 1.0);

        // From (C.32)
        final var e2 = e * e;
        final var f2 = f * f;
        final var q = 2.0 * (e2 - f2);

        // From (C.33)
        final var p3 = p * p * p;
        final var q2 = q * q;
        final var d = p3 + q2;

        // From (C.34)
        final var sqrtD = Math.sqrt(d);
        final var exp = 1.0 / 3.0;
        final var v = Math.pow(sqrtD - q, exp) - Math.pow(sqrtD + q, exp);

        // From (C.35)
        final var g = 0.5 * (Math.sqrt(e2 + v) + e);

        // From (C.36)
        final var g2 = g * g;
        final var t = Math.sqrt(g2 + (f - v * g) / (2.0 * g - e)) - g;

        // From (C.37)
        final var t2 = t * t;
        final var latitude = Math.signum(z) * Math.atan((1.0 - t2) / (2.0 * t * Math.sqrt(1.0 - ecc2)));

        // From (C.38)
        final var height = (beta - EARTH_EQUATORIAL_RADIUS_WGS84 * t) * Math.cos(latitude)
                + (z - Math.signum(z) * EARTH_EQUATORIAL_RADIUS_WGS84 * Math.sqrt(1.0 - ecc2))
                * Math.sin(latitude);

        try {
            // Calculate ECEF to NED coordinate transformation matrix
            final var cen = CoordinateTransformation.ecefToNedMatrix(latitude, longitude);

            // Transform velocity using (2.73)
            final var vx = source.getVx();
            final var vy = source.getVy();
            final var vz = source.getVz();
            final var vEbe = new Matrix(ECIorECEFFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbe.setElementAtIndex(0, vx);
            vEbe.setElementAtIndex(1, vy);
            vEbe.setElementAtIndex(2, vz);

            final var vEbn = cen.multiplyAndReturnNew(vEbe);
            final var vn = vEbn.getElementAtIndex(0);
            final var ve = vEbn.getElementAtIndex(1);
            final var vd = vEbn.getElementAtIndex(2);

            // Transform attitude using (2.15)
            final var cbe = source.getCoordinateTransformation().getMatrix();
            cen.multiply(cbe); // cen is now cbn

            final var c = new CoordinateTransformation(cen, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            // Set result
            destination.setLatitude(latitude);
            destination.setLongitude(longitude);
            destination.setHeight(height);

            destination.setVn(vn);
            destination.setVe(ve);
            destination.setVd(vd);

            destination.setCoordinateTransformation(c);

        } catch (final WrongSizeException | InvalidRotationMatrixException
                | InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }
}
