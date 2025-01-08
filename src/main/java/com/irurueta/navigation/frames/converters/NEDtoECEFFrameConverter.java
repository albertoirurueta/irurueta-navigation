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
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;

/**
 * Converts from NED frame to ECEF frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/NED_to_ECEF.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/NED_to_ECEF.m
 * </a>
 */
public class NEDtoECEFFrameConverter implements FrameConverter<NEDFrame, ECEFFrame> {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Converts source NED frame to a new ECEF frame instance.
     *
     * @param source source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public ECEFFrame convertAndReturnNew(final NEDFrame source) {
        final var result = new ECEFFrame();
        convert(source, result);
        return result;
    }

    /**
     * Converts source NED frame to destination ECEF frame.
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    @Override
    public void convert(final NEDFrame source, final ECEFFrame destination) {
        convertNEDtoECEF(source, destination);
    }

    /**
     * Gets source frame type.
     *
     * @return source frame type.
     */
    @Override
    public FrameType getSourceType() {
        return FrameType.LOCAL_NAVIGATION_FRAME;
    }

    /**
     * Gets destination frame type.
     *
     * @return destination frame type.
     */
    @Override
    public FrameType getDestinationType() {
        return FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME;
    }

    /**
     * Converts source NED frame to a new ECEF frame instance.
     *
     * @param source source frame to convert from.
     * @return a new destination frame instance.
     */
    public static ECEFFrame convertNEDtoECEFAndReturnNew(final NEDFrame source) {
        final var result = new ECEFFrame();
        convertNEDtoECEF(source, result);
        return result;
    }

    /**
     * Converts source NED frame to destination ECEF frame.
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void convertNEDtoECEF(final NEDFrame source, final ECEFFrame destination) {
        try {
            final var latitude = source.getLatitude();
            final var longitude = source.getLongitude();
            final var height = source.getHeight();

            final var cosLat = Math.cos(latitude);
            final var sinLat = Math.sin(latitude);
            final var cosLong = Math.cos(longitude);
            final var sinLong = Math.sin(longitude);

            // Calculate transverse radius of curvature using (2.105)
            final var eSinLat = EARTH_ECCENTRICITY * sinLat;
            final var eSinLat2 = eSinLat * eSinLat;
            var re = EARTH_EQUATORIAL_RADIUS_WGS84 / Math.sqrt(1.0 - eSinLat2);

            // Convert position using (2.112)
            final var e2 = EARTH_ECCENTRICITY * EARTH_ECCENTRICITY;
            final var x = (re + height) * cosLat * cosLong;
            final var y = (re + height) * cosLat * sinLong;
            final var z = ((1.0 - e2) * re + height) * sinLat;

            // Calculate NED to ECEF coordinate transformation matrix
            final var cne = CoordinateTransformation.nedToEcefMatrix(latitude, longitude);

            // Transform velocity using (2.73)
            final var vn = source.getVn();
            final var ve = source.getVe();
            final var vd = source.getVd();
            final var vEbn = new Matrix(NEDFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbn.setElementAtIndex(0, vn);
            vEbn.setElementAtIndex(1, ve);
            vEbn.setElementAtIndex(2, vd);

            final var vEbe = cne.multiplyAndReturnNew(vEbn);
            final var vx = vEbe.getElementAtIndex(0);
            final var vy = vEbe.getElementAtIndex(1);
            final var vz = vEbe.getElementAtIndex(2);

            // Transform attitude using (2.15)
            final var cbn = source.getCoordinateTransformation().getMatrix();
            cne.multiply(cbn); // cne is now cbe

            final var c = new CoordinateTransformation(cne, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            // set result
            destination.setX(x);
            destination.setY(y);
            destination.setZ(z);

            destination.setVx(vx);
            destination.setVy(vy);
            destination.setVz(vz);

            destination.setCoordinateTransformation(c);

        } catch (final WrongSizeException | InvalidRotationMatrixException |
                InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }
}
