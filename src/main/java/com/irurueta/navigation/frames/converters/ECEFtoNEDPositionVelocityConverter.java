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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;

/**
 * Converts cartesian to curvilinear position and velocity resolving axes
 * from ECEF to NED.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/pv_ECEF_to_NED.m
 */
public class ECEFtoNEDPositionVelocityConverter {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Converts cartesian to curvilinear position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param sourcePosition      source position resolved on ECEF frame.
     * @param sourceVelocity      source velocity resolved on ECEF frame.
     * @param destinationPosition instance where position resolved on NED frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on NED frame will
     *                            be stored.
     */
    public void convert(final ECEFPosition sourcePosition,
                        final ECEFVelocity sourceVelocity,
                        final NEDPosition destinationPosition,
                        final NEDVelocity destinationVelocity) {
        convertECEFtoNED(sourcePosition, sourceVelocity, destinationPosition,
                destinationVelocity);
    }

    /**
     * Converts cartesian to curvilinear position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param x                   x cartesian coordinate of body frame expressed in meters (m).
     * @param y                   y cartesian coordinate of body frame expressed in meters (m).
     * @param z                   z cartesian coordinate of body frame expressed in meters (m).
     * @param vx                  x coordinate of body velocity expressed in meters per second (m/s).
     * @param vy                  y coordinate of body velocity expressed in meters per second (m/s).
     * @param vz                  z coordinate of body velocity expressed in meters per second (m/s).
     * @param destinationPosition instance where position resolved on NED frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on NED frame will
     *                            be stored.
     */
    public void convert(final double x, final double y, final double z,
                        final double vx, final double vy, final double vz,
                        final NEDPosition destinationPosition,
                        final NEDVelocity destinationVelocity) {
        convertECEFtoNED(x, y, z, vx, vy, vz, destinationPosition,
                destinationVelocity);
    }

    /**
     * Converts cartesian to curvilinear position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param sourcePosition      source position resolved on ECEF frame.
     * @param sourceVelocity      source velocity resolved on ECEF frame.
     * @param destinationPosition instance where position resolved on NED frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on NED frame will
     *                            be stored.
     */
    public static void convertECEFtoNED(final ECEFPosition sourcePosition,
                                        final ECEFVelocity sourceVelocity,
                                        final NEDPosition destinationPosition,
                                        final NEDVelocity destinationVelocity) {
        convertECEFtoNED(sourcePosition.getX(), sourcePosition.getY(),
                sourcePosition.getZ(), sourceVelocity.getVx(), sourceVelocity.getVy(),
                sourceVelocity.getVz(), destinationPosition, destinationVelocity);
    }

    /**
     * Converts cartesian to curvilinear position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param x                   x cartesian coordinate of body frame expressed in meters (m).
     * @param y                   y cartesian coordinate of body frame expressed in meters (m).
     * @param z                   z cartesian coordinate of body frame expressed in meters (m).
     * @param vx                  x coordinate of body velocity expressed in meters per second (m/s).
     * @param vy                  y coordinate of body velocity expressed in meters per second (m/s).
     * @param vz                  z coordinate of body velocity expressed in meters per second (m/s).
     * @param destinationPosition instance where position resolved on NED frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on NED frame will
     *                            be stored.
     */
    public static void convertECEFtoNED(final double x, final double y, final double z,
                                        final double vx, final double vy, final double vz,
                                        final NEDPosition destinationPosition,
                                        final NEDVelocity destinationVelocity) {

        try {
            // Convert position using Borlowski closed-form exact solution
            // From (2.113)
            final double longitude = Math.atan2(y, x);

            // From (C.29) and (C.30)
            final double ecc2 = EARTH_ECCENTRICITY * EARTH_ECCENTRICITY;
            final double k1 = Math.sqrt(1.0 - ecc2) * Math.abs(z);
            final double k2 = ecc2 * EARTH_EQUATORIAL_RADIUS_WGS84;
            final double beta = Math.sqrt(x * x + y * y);
            final double e = (k1 - k2) / beta;
            final double f = (k1 + k2) / beta;

            // From (C.31)
            final double p = 4.0 / 3.0 * (e * f + 1.0);

            // From (C.32)
            final double e2 = e * e;
            final double f2 = f * f;
            final double q = 2.0 * (e2 - f2);

            // From (C.33)
            final double p3 = p * p * p;
            final double q2 = q * q;
            final double d = p3 + q2;

            // From (C.34)
            final double v = Math.pow(Math.sqrt(d) - q, 1.0 / 3.0)
                    - Math.pow(Math.sqrt(d) + q, 1.0 / 3.0);

            // From (C.35)
            final double g = 0.5 * (Math.sqrt(e2 + v) + e);

            // From (C.36)
            final double g2 = g * g;
            final double t = Math.sqrt(g2 + (f - v * g) / (2.0 * g - e)) - g;

            // From (C.37)
            final double t2 = t * t;
            final double latitude = Math.signum(z) * Math.atan((1 - t2)
                    / (2.0 * t * Math.sqrt(1.0 - ecc2)));

            // From (C.38)
            final double height = (beta - EARTH_EQUATORIAL_RADIUS_WGS84 * t)
                    * Math.cos(latitude) + (z - Math.signum(z)
                    * EARTH_EQUATORIAL_RADIUS_WGS84 * Math.sqrt(1.0 - ecc2))
                    * Math.sin(latitude);

            // Calculate ECEF to NED coordinate transformation matrix
            final Matrix cen = CoordinateTransformation.ecefToNedMatrix(latitude, longitude);

            // Transform velocity using (2.73)
            final Matrix vEbe = new Matrix(ECEFFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbe.setElementAtIndex(0, vx);
            vEbe.setElementAtIndex(1, vy);
            vEbe.setElementAtIndex(2, vz);

            final Matrix vEbn = cen.multiplyAndReturnNew(vEbe);
            final double vn = vEbn.getElementAtIndex(0);
            final double ve = vEbn.getElementAtIndex(1);
            final double vd = vEbn.getElementAtIndex(2);

            destinationPosition.setCoordinates(latitude, longitude, height);
            destinationVelocity.setCoordinates(vn, ve, vd);

        } catch (WrongSizeException ignore) {
            // never happens
        }
    }
}
