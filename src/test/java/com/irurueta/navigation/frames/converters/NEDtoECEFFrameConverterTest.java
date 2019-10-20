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

public class NEDtoECEFFrameConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {

        assertEquals(NEDtoECEFFrameConverter.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(NEDtoECEFFrameConverter.EARTH_ECCENTRICITY, Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    public void testConvertAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final NEDtoECEFFrameConverter converter1 = new NEDtoECEFFrameConverter();

            final ECEFFrame ecefFrame = converter1.convertAndReturnNew(nedFrame1);

            // convert back to NED
            final ECEFtoNEDFrameConverter converter2 = new ECEFtoNEDFrameConverter();

            final NEDFrame nedFrame2 = converter2.convertAndReturnNew(ecefFrame);

            // check
            if (Math.abs(nedFrame1.getLatitude() - nedFrame2.getLatitude()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);

            if (Math.abs(nedFrame1.getVn() - nedFrame2.getVn()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);

            if (Math.abs(nedFrame1.getVd() - nedFrame2.getVd()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformationMatrix().getSourceType(),
                    nedFrame2.getCoordinateTransformationMatrix().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformationMatrix().getDestinationType(),
                    nedFrame2.getCoordinateTransformationMatrix().getDestinationType());

            final Quaternion q1 = new Quaternion();
            nedFrame1.getCoordinateTransformationMatrix().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            nedFrame2.getCoordinateTransformationMatrix().asRotation(q2);
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

            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final NEDtoECEFFrameConverter converter1 = new NEDtoECEFFrameConverter();

            final ECEFFrame ecefFrame = new ECEFFrame();
            converter1.convert(nedFrame1, ecefFrame);

            // convert back to NED
            final ECEFtoNEDFrameConverter converter2 = new ECEFtoNEDFrameConverter();

            final NEDFrame nedFrame2 = new NEDFrame();
            converter2.convert(ecefFrame, nedFrame2);

            // check
            if (Math.abs(nedFrame1.getLatitude() - nedFrame2.getLatitude()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);

            if (Math.abs(nedFrame1.getVn() - nedFrame2.getVn()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);

            if (Math.abs(nedFrame1.getVd() - nedFrame2.getVd()) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformationMatrix().getSourceType(),
                    nedFrame2.getCoordinateTransformationMatrix().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformationMatrix().getDestinationType(),
                    nedFrame2.getCoordinateTransformationMatrix().getDestinationType());

            final Quaternion q1 = new Quaternion();
            nedFrame1.getCoordinateTransformationMatrix().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            nedFrame2.getCoordinateTransformationMatrix().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
}
