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
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;

/**
 * Converts curvilinear to cartesian position and velocity resolving
 * axes from NED to ECEF.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/pv_NED_to_ECEF.m
 */
public class NEDtoECEFPositionVelocityConverter {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Converts curvilinear to cartesian position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param sourcePosition      source position resolved on NED frame.
     * @param sourceVelocity      source velocity resolved on NED frame.
     * @param destinationPosition instance where position resolved on ECEF frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on ECEF frame will
     *                            be stored.
     */
    public void convert(final NEDPosition sourcePosition,
                        final NEDVelocity sourceVelocity,
                        final ECEFPosition destinationPosition,
                        final ECEFVelocity destinationVelocity) {
        convertNEDtoECEF(sourcePosition, sourceVelocity, destinationPosition,
                destinationVelocity);
    }

    /**
     * Converts curvilinear to cartesian position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param latitude            latitude expressed in radians (rad).
     * @param longitude           longitude expressed in radians (rad).
     * @param height              height expressed in meters (m).
     * @param vn                  north coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param ve                  east coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param vd                  down coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param destinationPosition instance where position resolved on ECEF frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on ECEF frame will
     *                            be stored.
     */
    public void convert(final double latitude, final double longitude,
                        final double height, final double vn,
                        final double ve, final double vd,
                        final ECEFPosition destinationPosition,
                        final ECEFVelocity destinationVelocity) {
        convertNEDtoECEF(latitude, longitude, height, vn, ve, vd,
                destinationPosition, destinationVelocity);
    }

    /**
     * Converts curvilinear to cartesian position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param sourcePosition      source position resolved on NED frame.
     * @param sourceVelocity      source velocity resolved on NED frame.
     * @param destinationPosition instance where position resolved on ECEF frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on ECEF frame will
     *                            be stored.
     */
    public static void convertNEDtoECEF(final NEDPosition sourcePosition,
                                        final NEDVelocity sourceVelocity,
                                        final ECEFPosition destinationPosition,
                                        final ECEFVelocity destinationVelocity) {
        final double latitude = sourcePosition.getLatitude();
        final double longitude = sourcePosition.getLongitude();
        final double height = sourcePosition.getHeight();

        final double vn = sourceVelocity.getVn();
        final double ve = sourceVelocity.getVe();
        final double vd = sourceVelocity.getVd();

        convertNEDtoECEF(latitude, longitude, height, vn, ve, vd,
                destinationPosition, destinationVelocity);
    }

    /**
     * Converts curvilinear to cartesian position and velocity resolving axes
     * from NED to ECEF.
     *
     * @param latitude            latitude expressed in radians (rad).
     * @param longitude           longitude expressed in radians (rad).
     * @param height              height expressed in meters (m).
     * @param vn                  north coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param ve                  east coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param vd                  down coordinate of velocity of body frame expressed
     *                            in meters per second (m/s).
     * @param destinationPosition instance where position resolved on ECEF frame will
     *                            be stored.
     * @param destinationVelocity instance where velocity resolved on ECEF frame will
     *                            be stored.
     */
    public static void convertNEDtoECEF(final double latitude, final double longitude,
                                        final double height, final double vn,
                                        final double ve, final double vd,
                                        final ECEFPosition destinationPosition,
                                        final ECEFVelocity destinationVelocity) {
        try {

            // Calculate transverse radius of curvature using (2.105)
            final double re = EARTH_EQUATORIAL_RADIUS_WGS84
                    / Math.sqrt(1.0 - Math.pow(EARTH_ECCENTRICITY * Math.sin(latitude), 2.0));

            // Convert position using (2.112)
            final double cosLat = Math.cos(latitude);
            final double sinLat = Math.sin(latitude);
            final double cosLong = Math.cos(longitude);
            final double sinLong = Math.sin(longitude);

            final double x = (re + height) * cosLat * cosLong;
            final double y = (re + height) * cosLat * sinLong;
            final double z = ((1.0 - EARTH_ECCENTRICITY * EARTH_ECCENTRICITY) * re + height) * sinLat;

            destinationPosition.setCoordinates(x, y, z);

            // Calculate NED to ECEF coordinate transformation matrix
            final Matrix cne = CoordinateTransformation.nedToEcefMatrix(latitude, longitude);

            // Transform velocity using (2.73)
            final Matrix vEbn = new Matrix(NEDFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbn.setElementAtIndex(0, vn);
            vEbn.setElementAtIndex(1, ve);
            vEbn.setElementAtIndex(2, vd);

            final Matrix vEbe = cne.multiplyAndReturnNew(vEbn);
            final double vx = vEbe.getElementAtIndex(0);
            final double vy = vEbe.getElementAtIndex(1);
            final double vz = vEbe.getElementAtIndex(2);

            destinationVelocity.setCoordinates(vx, vy, vz);
        } catch (WrongSizeException ignore) {
            // never happens
        }
    }
}
