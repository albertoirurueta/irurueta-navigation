package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECEFFrameTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    @Test
    public void testConstants() {

        assertEquals(ECEFFrame.NUM_POSITION_COORDINATES, 3);
        assertEquals(ECEFFrame.NUM_VELOCITY_COORDINATES, 3);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        ECEFFrame frame = new ECEFFrame();

        // check
        assertEquals(frame.getX(), 0.0, 0.0);
        assertEquals(frame.getY(), 0.0, 0.0);
        assertEquals(frame.getZ(), 0.0, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        CoordinateTransformationMatrix c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with coordinate transformation matrix
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformationMatrix c1 = new CoordinateTransformationMatrix(
                m, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        frame = new ECEFFrame(c1);

        // check
        assertEquals(frame.getX(), 0.0, 0.0);
        assertEquals(frame.getY(), 0.0, 0.0);
        assertEquals(frame.getZ(), 0.0, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        CoordinateTransformationMatrix c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(new CoordinateTransformationMatrix(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with cartesian position coordinates
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        frame = new ECEFFrame(x, y, z);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with cartesian position and velocity coordinates
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        frame = new ECEFFrame(x, y, z, vx, vy, vz);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformationMatrix());

        c = frame.getCoordinateTransformationMatrix();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with cartesian position coordinates and coordinate transformation matrix
        frame = new ECEFFrame(x, y, z, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(x, y, z, new CoordinateTransformationMatrix(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with cartesian position and velocity coordinates, and with coordinate transformation matrix
        frame = new ECEFFrame(x, y, z, vx, vy, vz, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        c2 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(x, y, z, vx, vy, vz,
                    new CoordinateTransformationMatrix(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with another ECEF frame
        frame = new ECEFFrame(x, y, z, vx, vy, vz, c1);
        final ECEFFrame frame2 = new ECEFFrame(frame);

        // check
        assertEquals(frame.getX(), frame2.getX(), 0.0);
        assertEquals(frame.getY(), frame2.getY(), 0.0);
        assertEquals(frame.getZ(), frame2.getZ(), 0.0);

        assertEquals(frame.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame.getVz(), frame2.getVz(), 0.0);

        assertEquals(frame.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testGetSetX() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getX(), 0.0, 0.0);

        // set new value
        frame.setX(x);

        // check
        assertEquals(frame.getX(), x, 0.0);
    }

    @Test
    public void testGetSetY() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getY(), 0.0, 0.0);

        // set new value
        frame.setY(y);

        // check
        assertEquals(frame.getY(), y, 0.0);
    }

    @Test
    public void testGetSetZ() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getZ(), 0.0, 0.0);

        // set new value
        frame.setZ(z);

        // check
        assertEquals(frame.getZ(), z, 0.0);
    }

    @Test
    public void testSetCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial values
        assertEquals(frame.getX(), 0.0, 0.0);
        assertEquals(frame.getY(), 0.0, 0.0);
        assertEquals(frame.getZ(), 0.0, 0.0);

        // set new values
        frame.setCoordinates(x, y, z);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetPosition() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getPosition(), Point3D.create());

        // set new value
        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        frame.setPosition(position);

        // check
        assertEquals(frame.getPosition(), position);
    }

    @Test
    public void testGetSetVx() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getVx(), 0.0, 0.0);

        // set new value
        frame.setVx(vx);

        // check
        assertEquals(frame.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetVy() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getVy(), 0.0, 0.0);

        // set new value
        frame.setVy(vy);

        // check
        assertEquals(frame.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetVz() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getVz(), 0.0, 0.0);

        // set new value
        frame.setVz(vz);

        // check
        assertEquals(frame.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        // set new values
        frame.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        CoordinateTransformationMatrix c1 = frame.getCoordinateTransformationMatrix();
        assertEquals(c1.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c1.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
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
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

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
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final CoordinateTransformationMatrix c2 = new CoordinateTransformationMatrix(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final CoordinateTransformationMatrix c3 = new CoordinateTransformationMatrix(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(ECEFFrame.isValidCoordinateTransformationMatrix(c1));
        assertFalse(ECEFFrame.isValidCoordinateTransformationMatrix(c2));
        assertFalse(ECEFFrame.isValidCoordinateTransformationMatrix(c3));
    }

    @Test
    public void testCopyTo() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame();
        frame1.copyTo(frame2);

        // check
        assertEquals(frame1.getX(), frame2.getX(), 0.0);
        assertEquals(frame1.getY(), frame2.getY(), 0.0);
        assertEquals(frame1.getZ(), frame2.getZ(), 0.0);
        assertEquals(frame1.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame1.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame1.getVz(), frame2.getVz(), 0.0);
        assertEquals(frame1.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testCopyFrom() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame();
        frame2.copyFrom(frame1);

        // check
        assertEquals(frame1.getX(), frame2.getX(), 0.0);
        assertEquals(frame1.getY(), frame2.getY(), 0.0);
        assertEquals(frame1.getZ(), frame2.getZ(), 0.0);
        assertEquals(frame1.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame1.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame1.getVz(), frame2.getVz(), 0.0);
        assertEquals(frame1.getCoordinateTransformationMatrix(),
                frame2.getCoordinateTransformationMatrix());
    }

    @Test
    public void testHashCode() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame3 = new ECEFFrame();

        assertEquals(frame1.hashCode(), frame2.hashCode());
        assertNotEquals(frame1.hashCode(), frame3.hashCode());
    }

    @Test
    public void testEquals() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame3 = new ECEFFrame();

        assertTrue(frame1.equals(frame1));
        assertTrue(frame1.equals(frame2));
        assertFalse(frame1.equals(frame3));
        assertFalse(frame1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(frame1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame3 = new ECEFFrame();

        assertTrue(frame1.equals(frame1, THRESHOLD));
        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertFalse(frame1.equals(frame3, THRESHOLD));
        assertFalse(frame1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

        final Object frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }
}
