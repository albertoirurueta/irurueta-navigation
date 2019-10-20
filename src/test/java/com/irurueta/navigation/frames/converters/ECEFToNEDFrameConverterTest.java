package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.frames.CoordinateTransformationMatrix;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECEFToNEDFrameConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_Z_VALUE = -50.0;
    private static final double MAX_Z_VALUE = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {

        assertEquals(ECEFtoNEDFrameConverter.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(ECEFtoNEDFrameConverter.EARTH_ECCENTRICITY,
                Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    public void testConvertAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformationMatrix c = new CoordinateTransformationMatrix(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final ECEFtoNEDFrameConverter converter1 = new ECEFtoNEDFrameConverter();

            final NEDFrame nedFrame = converter1.convertAndReturnNew(ecefFrame1);

            // convert back to ECEF
            final NEDtoECEFFrameConverter converter2 = new NEDtoECEFFrameConverter();

            final ECEFFrame ecefFrame2 = converter2.convertAndReturnNew(nedFrame);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);

            if (Math.abs(ecefFrame1.getZ() - ecefFrame2.getZ()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformationMatrix().getSourceType(),
                    ecefFrame2.getCoordinateTransformationMatrix().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformationMatrix().getDestinationType(),
                    ecefFrame2.getCoordinateTransformationMatrix().getDestinationType());

            final Quaternion q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformationMatrix().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformationMatrix().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testConvert() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformationMatrix c = new CoordinateTransformationMatrix(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final ECEFtoNEDFrameConverter converter1 = new ECEFtoNEDFrameConverter();

            final NEDFrame nedFrame = new NEDFrame();
            converter1.convert(ecefFrame1, nedFrame);

            // convert back to ECEF
            final NEDtoECEFFrameConverter converter2 = new NEDtoECEFFrameConverter();

            final ECEFFrame ecefFrame2 = new ECEFFrame();
            converter2.convert(nedFrame, ecefFrame2);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);

            if (Math.abs(ecefFrame1.getZ() - ecefFrame2.getZ()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformationMatrix().getSourceType(),
                    ecefFrame2.getCoordinateTransformationMatrix().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformationMatrix().getDestinationType(),
                    ecefFrame2.getCoordinateTransformationMatrix().getDestinationType());

            final Quaternion q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformationMatrix().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformationMatrix().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
}
