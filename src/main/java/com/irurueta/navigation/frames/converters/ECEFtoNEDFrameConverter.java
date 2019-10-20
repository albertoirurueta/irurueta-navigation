package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.frames.CoordinateTransformationMatrix;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;

/**
 * Converts from ECEF frame to NED frame.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition".
 */
@SuppressWarnings("WeakerAccess")
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
        final NEDFrame result = new NEDFrame();
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
        final NEDFrame result = new NEDFrame();
        convertECEFtoNED(source, result);
        return result;
    }

    /**
     * Converts source ECEF frame to destination NED frame.
     *
     * @param source      source frame to convert from.
     * @param destination destination frame instance to convert to.
     */
    public static void convertECEFtoNED(final ECEFFrame source, final NEDFrame destination) {
        final double x = source.getX();
        final double y = source.getY();
        final double z = source.getZ();

        // Convert position using Borkowski closed-form exact solution from (2.113).
        final double longitude = Math.atan2(y, x);

        // From (C.29) and (C.30)
        final double ecc2 = EARTH_ECCENTRICITY * EARTH_ECCENTRICITY;
        final double k1 = Math.sqrt(1.0 - ecc2) * Math.abs(z);
        final double k2 = ecc2 * EARTH_EQUATORIAL_RADIUS_WGS84;

        final double x2 = x * x;
        final double y2 = y * y;
        final double beta = Math.sqrt(x2 + y2);

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
        final double srqtD = Math.sqrt(d);
        final double exp = 1.0 / 3.0;
        final double v = Math.pow(srqtD - q, exp) - Math.pow(srqtD + q, exp);

        // From (C.35)
        final double g = 0.5 * (Math.sqrt(e2 + v) + e);

        // From (C.36)
        final double g2 = g * g;
        final double t = Math.sqrt(g2 + (f - v * g) / (2.0 * g - e)) - g;

        // From (C.37)
        final double t2 = t * t;
        final double latitude = Math.signum(z) * Math.atan((1.0 - t2) / (2.0 * t * Math.sqrt(1.0 - e2)));

        // From (C.38)
        final double height = (beta - EARTH_EQUATORIAL_RADIUS_WGS84 * t) * Math.cos(latitude)
                + (z - Math.signum(z) * EARTH_EQUATORIAL_RADIUS_WGS84 * Math.sqrt(1.0 - e2))
                * Math.sin(latitude);

        try {
            // Calculate ECEF to NED coordinate transformation matrix
            final Matrix cen = CoordinateTransformationMatrix.ecefToNedMatrix(latitude, longitude);

            // Transform velocity using (2.73)
            final double vx = source.getVx();
            final double vy = source.getVy();
            final double vz = source.getVz();
            final Matrix vEbe = new Matrix(ECEFFrame.NUM_VELOCITY_COORDINATES, 1);
            vEbe.setElementAtIndex(0, vx);
            vEbe.setElementAtIndex(1, vy);
            vEbe.setElementAtIndex(2, vz);

            final Matrix vEbn = cen.multiplyAndReturnNew(vEbe);
            final double vn = vEbn.getElementAtIndex(0);
            final double ve = vEbn.getElementAtIndex(1);
            final double vd = vEbn.getElementAtIndex(2);

            // Transform attitude using (2.15)
            final Matrix cbe = source.getCoordinateTransformationMatrix().getMatrix();
            cen.multiply(cbe); // cen is now cbn

            final CoordinateTransformationMatrix c = new CoordinateTransformationMatrix(cen, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            // Set result
            destination.setLatitude(latitude);
            destination.setLongitude(longitude);
            destination.setHeight(height);

            destination.setVn(vn);
            destination.setVe(ve);
            destination.setVd(vd);

            destination.setCoordinateTransformationMatrix(c);

        } catch (WrongSizeException | InvalidRotationMatrixException
                | InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }
}
