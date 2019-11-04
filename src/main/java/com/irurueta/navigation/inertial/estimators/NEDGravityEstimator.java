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

import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

/**
 * Calculates acceleration due to gravity resolved about north, east
 * and down axes of a NED frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
@SuppressWarnings("WeakerAccess")
public class NEDGravityEstimator {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * The polar radius of WGS84 ellipsoid (6356752.31425 m) defining Earth's shape.
     */
    public static final double EARTH_POLAR_RADIUS_WGS84 =
            Constants.EARTH_POLAR_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = 0.0818191908425;

    /**
     * The flattening of WGS84 ellipsoid (1 / 298.257223563).
     */
    public static final double EARTH_FLATTENING_WGS84 = 1 / 298.257223563;

    /**
     * WGS84 Earth gravitational constant expressed in m^3 * s^-2
     */
    public static final double EARTH_GRAVITATIONAL_CONSTANT = 3.986004418E14;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = 7.292115E-5;

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude expressed in radians (rad).
     * @param height   height expressed in meters (m).
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final double latitude, final double height, final NEDGravity result) {
        estimateGravity(latitude, height, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude expressed in radians (rad).
     * @param height   height expressed in meters (m).
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public NEDGravity estimateAndReturnNew(final double latitude, final double height) {
        return estimateGravityAndReturnNew(latitude, height);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param frame  a NED frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final NEDFrame frame, final NEDGravity result) {
        estimateGravity(frame, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param frame a NED frame containing a given position.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public NEDGravity estimateAndReturnNew(final NEDFrame frame) {
        return estimateGravityAndReturnNew(frame);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude.
     * @param height   height.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public void estimate(final Angle latitude, final Distance height, final NEDGravity result) {
        estimateGravity(latitude, height, result);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude.
     * @param height   height.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public NEDGravity estimateAndReturnNew(final Angle latitude, final Distance height) {
        return estimateGravityAndReturnNew(latitude, height);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude expressed in radians (rad).
     * @param height   height expressed in meters (m).
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final double latitude, final double height, final NEDGravity result) {
        // Calculate surface gravity using the Somigliana model (2.134)
        final double sinsqL = Math.pow(Math.sin(latitude), 2.0);
        final double e2 = EARTH_ECCENTRICITY * EARTH_ECCENTRICITY;
        final double g0 = 9.7803253359 * (1.0 + 0.001931853 * sinsqL) / Math.sqrt(1.0 - e2 * sinsqL);

        // Calculate north gravity using (2.140)
        final double gn = -8.08E-9 * height * Math.sin(2.0 * latitude);

        // Calculate down gravity using (2.139)
        final double omegaIe2 = EARTH_ROTATION_RATE * EARTH_ROTATION_RATE;
        final double r02 = EARTH_EQUATORIAL_RADIUS_WGS84 * EARTH_EQUATORIAL_RADIUS_WGS84;
        final double height2 = height * height;
        final double gd = g0 * (1.0 - (2.0 / EARTH_EQUATORIAL_RADIUS_WGS84) * (1.0 + EARTH_FLATTENING_WGS84 *
                (1.0 - 2.0 * sinsqL) + (omegaIe2 * r02 * EARTH_POLAR_RADIUS_WGS84 / EARTH_GRAVITATIONAL_CONSTANT)) *
                height + (3.0 * height2 / r02));

        // NOTE: East gravity is zero
        result.setCoordinates(gn, gd);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude expressed in radians (rad).
     * @param height   height expressed in meters (m).
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static NEDGravity estimateGravityAndReturnNew(final double latitude, final double height) {
        final NEDGravity result = new NEDGravity();
        estimateGravity(latitude, height, result);
        return result;
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param frame  a NED frame containing a given position.
     * @param result instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final NEDFrame frame, final NEDGravity result) {
        estimateGravity(frame.getLatitude(), frame.getHeight(), result);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param frame a NED frame containing a given position.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static NEDGravity estimateGravityAndReturnNew(final NEDFrame frame) {
        return estimateGravityAndReturnNew(frame.getLatitude(), frame.getHeight());
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude.
     * @param height   height.
     * @param result   instance where estimated acceleration due to gravity will be stored.
     */
    public static void estimateGravity(final Angle latitude, final Distance height, final NEDGravity result) {
        estimateGravity(convertAngle(latitude), convertDistance(height), result);
    }

    /**
     * Estimates acceleration due to gravity resolved about NED for a given position expressed in NED coordinates.
     *
     * @param latitude latitude.
     * @param height   height.
     * @return a new gravity instance containing estimated acceleration due to gravity.
     */
    public static NEDGravity estimateGravityAndReturnNew(final Angle latitude, final Distance height) {
        return estimateGravityAndReturnNew(convertAngle(latitude), convertDistance(height));
    }

    /**
     * Converts angle to radians.
     *
     * @param angle angle to be converted.
     * @return converted angle expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Converts distance to meters.
     *
     * @param distance distance to be converted.
     * @return converted distance expressed in meters.
     */
    private static double convertDistance(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }
}
