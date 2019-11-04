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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

/**
 * Calculates gravitational acceleration resolved about ECI-frame axes.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
@SuppressWarnings("WeakerAccess")
public class ECIGravitationEstimator {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * WGS84 Earth gravitational constant expressed in m^3 * s^-2
     */
    public static final double EARTH_GRAVITATIONAL_CONSTANT =
            Constants.EARTH_GRAVITATIONAL_CONSTANT;

    /**
     * WGS84 Earth's second gravitational constant.
     */
    public static final double EARTH_SECOND_GRAVITATIONAL_CONSTANT =
            Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT;

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final double x, final double y, final double z, final ECIGravitation result) {
        estimateGravitation(x, y, z, result);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public ECIGravitation estimateAndReturnNew(final double x, final double y, final double z) {
        return estimateGravitationAndReturnNew(x, y, z);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a position on a given ECI frame.
     *
     * @param frame  an ECI frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final ECIFrame frame, final ECIGravitation result) {
        estimateGravitation(frame, result);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a position on a given ECI frame.
     *
     * @param frame an ECI frame containing a given position.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public ECIGravitation estimateAndReturnNew(final ECIFrame frame) {
        return estimateGravitationAndReturnNew(frame);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a given position expressed in
     * ECI coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECI frame, resolved along
     *                 ECI-frame axes.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final Point3D position, final ECIGravitation result) {
        estimateGravitation(position, result);
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position expressed in
     * ECI coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECI frame, resolved along
     *                 ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public ECIGravitation estimateAndReturnNew(final Point3D position) {
        return estimateGravitationAndReturnNew(position);
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position expressed in
     * ECI coordinates.
     *
     * @param x      cartesian x coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param y      cartesian y coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param z      cartesian z coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final Distance x, final Distance y, final Distance z,
                         final ECIGravitation result) {
        estimateGravitation(x, y, z, result);
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position
     * expressed in ECI coordinates.
     *
     * @param x cartesian x coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param y cartesian y coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param z cartesian z coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public ECIGravitation estimateAndReturnNew(final Distance x, final Distance y, final Distance z) {
        return estimateGravitationAndReturnNew(x, y, z);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECI frame, resolved
     *               along ECI-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravitation(final double x, final double y, final double z, final ECIGravitation result) {

        // Calculate distance from center of the Earth
        final double magR = Math.sqrt(x * x + y * y + z * z);

        // If the input position is 0,0,0, produce a dummy output
        if (magR == 0.0) {
            result.setCoordinates(0.0, 0.0, 0.0);
        } else {
            final double zScale = 5.0 * Math.pow(z / magR, 2.0);
            final double tmp1 = -EARTH_GRAVITATIONAL_CONSTANT / Math.pow(magR, 3.0);
            final double tmp2 = 1.5 * EARTH_SECOND_GRAVITATIONAL_CONSTANT * Math.pow(
                    EARTH_EQUATORIAL_RADIUS_WGS84 / magR, 2.0);
            final double gx = tmp1 * (1.0 + tmp2 * (1.0 - zScale)) * x;
            final double gy = tmp1 * (1.0 + tmp2 * (1.0 - zScale)) * y;
            final double gz = tmp1 * (1.0 + tmp2 * (3.0 - zScale)) * z;

            result.setCoordinates(gx, gy, gz);
        }
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public static ECIGravitation estimateGravitationAndReturnNew(final double x, final double y, final double z) {
        final ECIGravitation result = new ECIGravitation();
        estimateGravitation(x, y, z, result);
        return result;
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a position on a given ECI frame.
     *
     * @param frame  an ECI frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravitation(final ECIFrame frame, final ECIGravitation result) {
        estimateGravitation(frame.getX(), frame.getY(), frame.getZ(), result);
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a position on a given ECI frame.
     *
     * @param frame an ECI frame containing a given position.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public static ECIGravitation estimateGravitationAndReturnNew(final ECIFrame frame) {
        return estimateGravitationAndReturnNew(frame.getX(), frame.getY(), frame.getZ());
    }

    /**
     * Estimates gravitational acceleration resolved about ECI-frame axes for a given position expressed in
     * ECI coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECI frame, resolved along
     *                 ECI-frame axes.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravitation(final Point3D position, final ECIGravitation result) {
        estimateGravitation(position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position expressed in
     * ECI coordinates.
     *
     * @param position cartesian body position expressed in meters (m) with respect ECI frame, resolved along
     *                 ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public static ECIGravitation estimateGravitationAndReturnNew(final Point3D position) {
        return estimateGravitationAndReturnNew(position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position
     * expressed in ECI coordinates.
     *
     * @param x      cartesian x coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param y      cartesian y coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param z      cartesian z coordinate of body position with respect ECI frame, resolved along
     *               ECI-frame axes.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravitation(final Distance x, final Distance y,
                                           final Distance z, final ECIGravitation result) {
        estimateGravitation(convertToMeters(x), convertToMeters(y), convertToMeters(z),
                result);
    }

    /**
     * Estimates gravitation acceleration resolved about ECI-frame axes for a given position
     * expressed in ECI coordinates.
     *
     * @param x cartesian x coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param y cartesian y coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @param z cartesian z coordinate of body position with respect ECI frame, resolved along
     *          ECI-frame axes.
     * @return a new gravitation instance containing estimated acceleration due to gravity.
     */
    public static ECIGravitation estimateGravitationAndReturnNew(final Distance x,
                                                                 final Distance y,
                                                                 final Distance z) {
        final ECIGravitation result = new ECIGravitation();
        estimateGravitation(x, y, z, result);
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
