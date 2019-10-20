package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class NEDFrameTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    @Test
    public void testConstants() {

        assertEquals(NEDFrame.NUM_VELOCITY_COORDINATES, 3);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        NEDFrame frame = new NEDFrame();

        // check
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        CoordinateTransformationMatrix c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with coordinate transformation matrix
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformationMatrix c1 = new CoordinateTransformationMatrix(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        frame = new NEDFrame(c1);

        // check
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        CoordinateTransformationMatrix c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(new CoordinateTransformationMatrix(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position coordinates
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        frame = new NEDFrame(latitude, longitude, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates and coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, height, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, height,
                    new CoordinateTransformationMatrix(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position and velocity coordinates, and with coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, height, vn, ve, vd,
                    new CoordinateTransformationMatrix(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with another NED frame
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);
        final NEDFrame frame2 = new NEDFrame(frame);

        // check
        assertEquals(frame.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame.getHeight(), frame2.getHeight(), 0.0);

        assertEquals(frame.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame.getVd(), frame2.getVd(), 0.0);

        assertEquals(frame.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testGetSetLatitude() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLatitude(), 0.0, 0.0);

        // set new value
        frame.setLatitude(latitude);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
    }

    @Test
    public void testGetSetLongitude() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLongitude(), 0.0, 0.0);

        // set new value
        frame.setLongitude(longitude);

        // check
        assertEquals(frame.getLongitude(), longitude, 0.0);
    }

    @Test
    public void testGetSetHeight() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new value
        frame.setHeight(height);

        // check
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testSetPosition() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        frame.setPosition(latitude, longitude, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testGetSetVn() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVn(), 0.0, 0.0);

        // set new value
        frame.setVn(vn);

        // check
        assertEquals(frame.getVn(), vn, 0.0);
    }

    @Test
    public void testGetSetVe() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVe(), 0.0, 0.0);

        // set new value
        frame.setVe(ve);

        // check
        assertEquals(frame.getVe(), ve, 0.0);
    }

    @Test
    public void testGetSetVd() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVd(), 0.0, 0.0);

        // set new value
        frame.setVd(vd);

        // check
        assertEquals(frame.getVd(), vd, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        // set new values
        frame.setVelocityCoordinates(vn, ve, vd);

        // check
        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final NEDFrame frame = new NEDFrame();

        // check initial value
        CoordinateTransformationMatrix c1 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c1.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c1.getMatrix(), Matrix.identity(3, 3));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformationMatrix c2 = new CoordinateTransformationMatrix(
                m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        frame.setCoordinateTransformationMatrix(c2);

        // check
        assertEquals(frame.getCoordinateTransformationMatrix(), c2);
        final CoordinateTransformationMatrix c3 = new CoordinateTransformationMatrix(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        frame.getCoordinateTransformationMatrix(c3);
        assertEquals(c2, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            frame.setCoordinateTransformationMatrix(new CoordinateTransformationMatrix(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
    }

    @Test
    public void testIsValidCoordinateTransformationMatrix() {
        final CoordinateTransformationMatrix c1 = new CoordinateTransformationMatrix(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformationMatrix c2 = new CoordinateTransformationMatrix(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final CoordinateTransformationMatrix c3 = new CoordinateTransformationMatrix(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(NEDFrame.isValidCoordinateTransformationMatrix(c1));
        assertFalse(NEDFrame.isValidCoordinateTransformationMatrix(c2));
        assertFalse(NEDFrame.isValidCoordinateTransformationMatrix(c3));
    }

    @Test
    public void testCopyTo() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame();
        frame1.copyTo(frame2);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testCopyFrom() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame();
        frame2.copyFrom(frame1);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testHashCode() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        assertEquals(frame1.hashCode(), frame2.hashCode());
        assertNotEquals(frame1.hashCode(), frame3.hashCode());
    }

    @Test
    public void testEquals() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        assertTrue(frame1.equals(frame1));
        assertTrue(frame1.equals(frame2));
        assertFalse(frame1.equals(frame3));
        assertFalse(frame1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(frame1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        assertTrue(frame1.equals(frame1, THRESHOLD));
        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertFalse(frame1.equals(frame3, THRESHOLD));
        assertFalse(frame1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

        final Object frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }
}
