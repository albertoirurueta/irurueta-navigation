package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Converts from ECI frame to ECEF frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
@SuppressWarnings("WeakerAccess")
public class ECItoECEFFrameConverter implements TimeIntervalFrameConverter<ECIFrame, ECEFFrame> {

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Converts source ECI frame to a new ECEF frame instance.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public ECEFFrame convertAndReturnNew(double timeInterval, final ECIFrame source) {
        return convertECItoECEFAndReturnNew(timeInterval, source);
    }

    /**
     * Converts source frame to a new destination frame instance.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    @Override
    public ECEFFrame convertAndReturnNew(final Time timeInterval, final ECIFrame source) {
        return convertECItoECEFAndReturnNew(timeInterval, source);
    }

    /**
     * Converts source ECI frame to destination ECEF frame.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    @Override
    public void convert(final double timeInterval, final ECIFrame source, final ECEFFrame destination) {
        convertECItoECEF(timeInterval, source, destination);
    }

    /**
     * Converts source frame to destination frame.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to covert to.
     */
    @Override
    public void convert(final Time timeInterval, final ECIFrame source, final ECEFFrame destination) {
        convertECItoECEF(timeInterval, source, destination);
    }

    /**
     * Gets source frame type.
     *
     * @return source frame type.
     */
    @Override
    public FrameType getSourceType() {
        return FrameType.EARTH_CENTERED_INERTIAL_FRAME;
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
     * Converts source ECI frame to a new ECEF frame instance.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    public static ECEFFrame convertECItoECEFAndReturnNew(final double timeInterval, final ECIFrame source) {
        final ECEFFrame result = new ECEFFrame();
        convertECItoECEF(timeInterval, source, result);
        return result;
    }

    /**
     * Convers source ECI frame to a new ECEF frame instance.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @return a new destination frame instance.
     */
    public static ECEFFrame convertECItoECEFAndReturnNew(final Time timeInterval, final ECIFrame source) {
        return convertECItoECEFAndReturnNew(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND), source);
    }

    /**
     * Converts source ECI frame to destination ECEF frame.
     *
     * @param timeInterval a time interval expressed in seconds (s).
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    public static void convertECItoECEF(final double timeInterval, final ECIFrame source, final ECEFFrame destination) {
        try {
            // Calculate ECEF to ECI coordinate transformation matrix using (2.145)
            final double alpha = EARTH_ROTATION_RATE * timeInterval;
            final Matrix cie = CoordinateTransformation.eciToEcefMatrixFromAngle(alpha);

            // Transform position using (2.146)
            final Matrix rIbi = new Matrix(ECIFrame.NUM_POSITION_COORDINATES, 1);
            rIbi.setElementAtIndex(0, source.getX());
            rIbi.setElementAtIndex(1, source.getY());
            rIbi.setElementAtIndex(2, source.getZ());

            final Matrix rEbe = cie.multiplyAndReturnNew(rIbi);

            destination.setCoordinates(rEbe.getElementAtIndex(0),
                    rEbe.getElementAtIndex(1), rEbe.getElementAtIndex(2));

            // Transform velocity using (2.145)
            final Matrix tmp = new Matrix(ECIFrame.NUM_POSITION_COORDINATES, 1);
            tmp.setElementAtIndex(0, -source.getY());
            tmp.setElementAtIndex(1, source.getX());
            tmp.setElementAtIndex(2, 0.0);
            tmp.multiplyByScalar(-EARTH_ROTATION_RATE);

            final Matrix vIbi = new Matrix(ECIFrame.NUM_VELOCITY_COORDINATES, 1);
            vIbi.setElementAtIndex(0, source.getVx());
            vIbi.setElementAtIndex(1, source.getVy());
            vIbi.setElementAtIndex(2, source.getVz());

            vIbi.add(tmp); // vIbi - omega * [-y;x;0]

            final Matrix vEbe = cie.multiplyAndReturnNew(vIbi);

            destination.setVelocityCoordinates(vEbe.getElementAtIndex(0),
                    vEbe.getElementAtIndex(1), vEbe.getElementAtIndex(2));

            // Transform attitude using (2.15)
            // cbe = cie * cbi
            final Matrix cbi = source.getCoordinateTransformationMatrix();
            cie.multiply(cbi);

            final CoordinateTransformation c = new CoordinateTransformation(cie,
                    FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            destination.setCoordinateTransformation(c);
        } catch (WrongSizeException | InvalidSourceAndDestinationFrameTypeException |
                InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Converts source ECI frame to destination ECEF frame.
     *
     * @param timeInterval a time interval.
     * @param source       source frame to convert from.
     * @param destination  destination frame instance to convert to.
     */
    public static void convertECItoECEF(final Time timeInterval, final ECIFrame source, final ECEFFrame destination) {
        convertECItoECEF(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND), source, destination);
    }
}
