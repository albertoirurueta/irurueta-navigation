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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.ECIorECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Converts from ECEF frame to ECI frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/ECEF_to_ECI.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/ECEF_to_ECI.m
 * </a>
 */
public class ECEFtoECIFrameConverter implements TimeIntervalFrameConverter<ECEFFrame, ECIFrame> {

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Converts source ECEF frame to a new ECI frame instance.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public ECIFrame convertAndReturnNew(final double timeInterval, final ECEFFrame source) {
        return convertECEFtoECIAndReturnNew(timeInterval, source);
    }

    /**
     * Converts source frame to a new destination frame instance.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public ECIFrame convertAndReturnNew(final Time timeInterval, final ECEFFrame source) {
        return convertECEFtoECIAndReturnNew(timeInterval, source);
    }

    /**
     * Converts source ECEF frame to destination ECI frame.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    @Override
    public void convert(final double timeInterval, final ECEFFrame source, final ECIFrame destination) {
        convertECEFtoECI(timeInterval, source, destination);
    }

    /**
     * Converts source frame to destination frame.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to covert to.
     */
    @Override
    public void convert(final Time timeInterval, final ECEFFrame source, final ECIFrame destination) {
        convertECEFtoECI(timeInterval, source, destination);
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
        return FrameType.EARTH_CENTERED_INERTIAL_FRAME;
    }

    /**
     * Converts source ECEF frame to a new ECI frame instance.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    public static ECIFrame convertECEFtoECIAndReturnNew(final double timeInterval, final ECEFFrame source) {
        final var result = new ECIFrame();
        convertECEFtoECI(timeInterval, source, result);
        return result;
    }

    /**
     * Converts source ECEF frame to a new ECI frame instance.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    public static ECIFrame convertECEFtoECIAndReturnNew(final Time timeInterval, final ECEFFrame source) {
        return convertECEFtoECIAndReturnNew(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND), source);
    }

    /**
     * Converts source ECEF frame to destination ECI frame.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void convertECEFtoECI(final double timeInterval, final ECEFFrame source, final ECIFrame destination) {
        try {
            // Calculate ECEF to ECI coordinate transformation matrix using (2.145)
            final var alpha = EARTH_ROTATION_RATE * timeInterval;
            final var cei = CoordinateTransformation.ecefToEciMatrixFromAngle(alpha);

            // Transform position using (2.146)
            final var rEbe = new Matrix(ECIorECEFFrame.NUM_POSITION_COORDINATES, 1);
            rEbe.setElementAtIndex(0, source.getX());
            rEbe.setElementAtIndex(1, source.getY());
            rEbe.setElementAtIndex(2, source.getZ());

            final var rIbi = cei.multiplyAndReturnNew(rEbe);

            destination.setCoordinates(rIbi.getElementAtIndex(0),
                    rIbi.getElementAtIndex(1), rIbi.getElementAtIndex(2));

            // Transform velocity using (2.145)
            final var tmp = new Matrix(ECIorECEFFrame.NUM_POSITION_COORDINATES, 1);
            tmp.setElementAtIndex(0, -source.getY());
            tmp.setElementAtIndex(1, source.getX());
            tmp.setElementAtIndex(2, 0.0);
            tmp.multiplyByScalar(EARTH_ROTATION_RATE);

            final var vEbe = new Matrix(ECIorECEFFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbe.setElementAtIndex(0, source.getVx());
            vEbe.setElementAtIndex(1, source.getVy());
            vEbe.setElementAtIndex(2, source.getVz());

            // vEbe + omega * [-y;x;0]
            vEbe.add(tmp);

            final var vIbi = cei.multiplyAndReturnNew(vEbe);

            destination.setVelocityCoordinates(vIbi.getElementAtIndex(0),
                    vIbi.getElementAtIndex(1), vIbi.getElementAtIndex(2));

            // Transform attitude using (2.15)
            // cbi = cei * cbe
            final var cbe = source.getCoordinateTransformationMatrix();
            cei.multiply(cbe);

            final var c = new CoordinateTransformation(cei,
                    FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);
            destination.setCoordinateTransformation(c);
        } catch (final WrongSizeException | InvalidSourceAndDestinationFrameTypeException |
                InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Converts source ECEF frame to destination ECI frame.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    public static void convertECEFtoECI(final Time timeInterval, final ECEFFrame source, final ECIFrame destination) {
        convertECEFtoECI(TimeConverter.convert(timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND), source, destination);
    }
}
