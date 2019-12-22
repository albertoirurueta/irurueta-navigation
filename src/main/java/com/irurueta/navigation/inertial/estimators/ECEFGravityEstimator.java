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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

/**
 * Calculates acceleration due to gravity resolved about ECEF frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Gravity_ECEF.m
 */
@SuppressWarnings("WeakerAccess")
public class ECEFGravityEstimator {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * WGS84 Earth gravitational constant expressed in m^3 * s^-2
     */
    public static final double EARTH_GRAVITATIONAL_CONSTANT = Constants.EARTH_GRAVITATIONAL_CONSTANT;

    /**
     * WGS84 Earth's second gravitational constant.
     */
    public static final double EARTH_SECOND_GRAVITATIONAL_CONSTANT = Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final double x, final double y, final double z, final ECEFGravity result) {
        estimateGravity(x, y, z, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public ECEFGravity estimateAndReturnNew(final double x, final double y, final double z) {
        return estimateGravityAndReturnNew(x, y, z);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a position on a given ECEF frame.
     *
     * @param frame  an ECEF frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final ECEFFrame frame, final ECEFGravity result) {
        estimateGravity(frame, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a position on a given ECEF frame.
     *
     * @param frame an ECEF frame containing a given position.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public ECEFGravity estimateAndReturnNew(final ECEFFrame frame) {
        return estimateGravityAndReturnNew(frame);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final Point3D position, final ECEFGravity result) {
        estimateGravity(position, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public ECEFGravity estimateAndReturnNew(final Point3D position) {
        return estimateGravityAndReturnNew(position);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x      cartesian x coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final Distance x, final Distance y, final Distance z, final ECEFGravity result) {
        estimateGravity(x, y, z, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x cartesian x coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y cartesian y coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z cartesian z coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public ECEFGravity estimateAndReturnNew(final Distance x, final Distance y, final Distance z) {
        return estimateGravityAndReturnNew(x, y, z);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final double x, final double y, final double z, final ECEFGravity result) {
        // Calculate distance from center of the Earth
        final double magR = Math.sqrt(x * x + y * y + z * z);

        if (magR == 0.0) {
            // If the input position is 0,0,0, produce a dummy output
            result.setCoordinates(0.0, 0.0, 0.0);
        } else {
            // Calculate gravitational acceleration using (2.142)
            final double zScale = 5.0 * Math.pow(z / magR, 2.0);
            final double tmp1 = -EARTH_GRAVITATIONAL_CONSTANT / Math.pow(magR, 3.0);
            final double tmp2 = 1.5 * EARTH_SECOND_GRAVITATIONAL_CONSTANT
                    * Math.pow(EARTH_EQUATORIAL_RADIUS_WGS84 / magR, 2.0);
            final double tmp3 = tmp1 * (1.0 + tmp2 * (1.0 - zScale));

            final double gammaX = tmp3 * x;
            final double gammaY = tmp3 * y;
            final double gammaz = tmp1 * (1.0 + tmp2 * (3.0 - zScale)) * z;

            // Add centripetal acceleration using (2.133)
            final double omega2 = EARTH_ROTATION_RATE * EARTH_ROTATION_RATE;
            final double gx = gammaX + omega2 * x;
            final double gy = gammaY + omega2 * y;

            result.setCoordinates(gx, gy, gammaz);
        }
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static ECEFGravity estimateGravityAndReturnNew(final double x, final double y, final double z) {
        final ECEFGravity result = new ECEFGravity();
        estimateGravity(x, y, z, result);
        return result;
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a position on a given ECEF frame.
     *
     * @param frame  an ECEF frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final ECEFFrame frame, final ECEFGravity result) {
        estimateGravity(frame.getX(), frame.getY(), frame.getZ(), result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a position on a given ECEF frame.
     *
     * @param frame an ECEF frame containing a given position.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static ECEFGravity estimateGravityAndReturnNew(final ECEFFrame frame) {
        return estimateGravityAndReturnNew(frame.getX(), frame.getY(), frame.getZ());
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final Point3D position, final ECEFGravity result) {
        estimateGravity(position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static ECEFGravity estimateGravityAndReturnNew(final Point3D position) {
        return estimateGravityAndReturnNew(position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x      cartesian x coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final Distance x, final Distance y,
                                       final Distance z, final ECEFGravity result) {
        estimateGravity(convertToMeters(x), convertToMeters(y), convertToMeters(z),
                result);
    }

    /**
     * Estimates acceleration due to gravity resolved about ECEF for a given position expressed in ECEF coordinates.
     *
     * @param x cartesian x coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y cartesian y coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z cartesian z coordinate of body position with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static ECEFGravity estimateGravityAndReturnNew(final Distance x,
                                                          final Distance y,
                                                          final Distance z) {
        final ECEFGravity result = new ECEFGravity();
        estimateGravity(x, y, z, result);
        return result;
    }

    /**
     * Converts distance to meters.
     *
     * @param distance distance to be converted.
     * @return converted distance expressed in meters.
     */
    private static double convertToMeters(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }
}
