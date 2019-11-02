package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
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

    private static final double ABSOLUTE_ERROR = 1e-8;

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

        assertEquals(frame.getPositionX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        CoordinateTransformation c = frame.getCoordinateTransformation();
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
        final CoordinateTransformation c1 = new CoordinateTransformation(
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

        assertEquals(frame.getPositionX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        CoordinateTransformation c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
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

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        frame = new ECEFFrame(position);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position coordinates
        final Distance positionX = new Distance(x, DistanceUnit.METER);
        final Distance positionY = new Distance(y, DistanceUnit.METER);
        final Distance positionZ = new Distance(z, DistanceUnit.METER);

        frame = new ECEFFrame(positionX, positionY, positionZ);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
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

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new ECEFFrame(position, vx, vy, vz);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and speed coordinates
        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        frame = new ECEFFrame(position, speedX, speedY, speedZ);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with cartesian position coordinates and speed coordinates
        frame = new ECEFFrame(x, y, z, speedX, speedY, speedZ);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new ECEFFrame(positionX, positionY, positionZ, vx, vy, vz);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and speed coordinates
        frame = new ECEFFrame(positionX, positionY, positionZ, speedX, speedY, speedZ);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
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

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(x, y, z, new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position and coordinate transformation matrix
        frame = new ECEFFrame(position, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(position, new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position coordinates and coordinate transformation matrix
        frame = new ECEFFrame(positionX, positionY, positionZ, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), 0.0, 0.0);
        assertEquals(frame.getVy(), 0.0, 0.0);
        assertEquals(frame.getVz(), 0.0, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(positionX, positionY, positionZ,
                    new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
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

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(x, y, z, vx, vy, vz,
                    new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position, velocity coordinates, and coordinate transformation matrix
        frame = new ECEFFrame(position, vx, vy, vz, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(position, vx, vy, vz,
                    new CoordinateTransformation(
                            FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position, speed coordinates and coordinate transformation matrix
        frame = new ECEFFrame(position, speedX, speedY, speedZ, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(position, speedX, speedY, speedZ,
                    new CoordinateTransformation(
                            FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with cartesian coordinates, speed coordinates and coordinate transformation matrix
        frame = new ECEFFrame(x, y, z, speedX, speedY, speedZ, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(x, y, z, speedX, speedY, speedZ,
                    new CoordinateTransformation(
                            FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position coordinates, velocity coordinates and coordinates transformation matrix
        frame = new ECEFFrame(positionX, positionY, positionZ, vx, vy, vz, c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(positionX, positionY, positionZ,
                    vx, vy, vz,
                    new CoordinateTransformation(
                            FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
        assertNull(frame);


        // test constructor with position coordinates, speed coordinates and coordinate transformation matrix
        frame = new ECEFFrame(positionX, positionY, positionZ, speedX, speedY, speedZ,
                c1);

        // check
        assertEquals(frame.getX(), x, 0.0);
        assertEquals(frame.getY(), y, 0.0);
        assertEquals(frame.getZ(), z, 0.0);

        assertEquals(frame.getVx(), vx, 0.0);
        assertEquals(frame.getVy(), vy, 0.0);
        assertEquals(frame.getVz(), vz, 0.0);

        assertEquals(frame.getPositionX().getValue().doubleValue(), x, 0.0);
        assertEquals(frame.getPositionY().getValue().doubleValue(), y, 0.0);
        assertEquals(frame.getPositionZ().getValue().doubleValue(), z, 0.0);

        assertEquals(frame.getSpeedX().getValue().doubleValue(), vx, 0.0);
        assertEquals(frame.getSpeedY().getValue().doubleValue(), vy, 0.0);
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), vz, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new ECEFFrame(positionX, positionY, positionZ,
                    speedX, speedY, speedZ,
                    new CoordinateTransformation(
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

        assertEquals(frame.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
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

        final Point3D position2 = new InhomogeneousPoint3D();
        frame.getPosition(position2);
        assertEquals(position, position2);
    }

    @Test
    public void testGetSetPositionX() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getPositionX().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance positionX1 = new Distance(x, DistanceUnit.METER);
        frame.setPositionX(positionX1);

        // check
        final Distance positionX2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionX(positionX2);
        final Distance positionX3 = frame.getPositionX();

        assertEquals(positionX1, positionX2);
        assertEquals(positionX1, positionX3);
    }

    @Test
    public void testGetSetPositionY() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getPositionY().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance positionY1 = new Distance(y, DistanceUnit.METER);
        frame.setPositionY(positionY1);

        // check
        final Distance positionY2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionY(positionY2);
        final Distance positionY3 = frame.getPositionY();

        assertEquals(positionY1, positionY2);
        assertEquals(positionY1, positionY3);
    }

    @Test
    public void testGetSetPositionZ() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getPositionZ().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance positionZ1 = new Distance(z, DistanceUnit.METER);
        frame.setPositionZ(positionZ1);

        // check
        final Distance positionZ2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionZ(positionZ2);
        final Distance positionZ3 = frame.getPositionZ();

        assertEquals(positionZ1, positionZ2);
        assertEquals(positionZ1, positionZ3);
    }

    @Test
    public void testSetPositionCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final Distance positionX = new Distance(x, DistanceUnit.METER);
        final Distance positionY = new Distance(y, DistanceUnit.METER);
        final Distance positionZ = new Distance(z, DistanceUnit.METER);

        final ECEFFrame frame = new ECEFFrame();

        frame.setPositionCoordinates(positionX, positionY, positionZ);

        // check
        assertEquals(frame.getPositionX(), positionX);
        assertEquals(frame.getPositionY(), positionY);
        assertEquals(frame.getPositionZ(), positionZ);
    }

    @Test
    public void testGetPositionNorm() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double norm = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0)
                + Math.pow(z, 2.0));

        final ECEFFrame frame = new ECEFFrame(x, y, z);

        assertEquals(frame.getPositionNorm(), norm, ABSOLUTE_ERROR);

        final Distance normDistance1 = new Distance(0.0, DistanceUnit.KILOMETER);
        frame.getPositionNormAsDistance(normDistance1);
        final Distance normDistance2 = frame.getPositionNormAsDistance();

        assertEquals(normDistance1.getValue().doubleValue(), norm, ABSOLUTE_ERROR);
        assertEquals(normDistance1.getUnit(), DistanceUnit.METER);
        assertEquals(normDistance1, normDistance2);
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
    public void testGetVelocityNorm() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double norm = Math.sqrt(Math.pow(vx, 2.0) + Math.pow(vy, 2.0)
                + Math.pow(vz, 2.0));

        final ECEFFrame frame = new ECEFFrame();
        frame.setVelocityCoordinates(vx, vy, vz);

        assertEquals(frame.getVelocityNorm(), norm, ABSOLUTE_ERROR);

        final Speed normSpeed1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getVelocityNormAsSpeed(normSpeed1);
        final Speed normSpeed2 = frame.getVelocityNormAsSpeed();

        assertEquals(normSpeed1.getValue().doubleValue(), norm, ABSOLUTE_ERROR);
        assertEquals(normSpeed1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    public void testGetSetSpeedX() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getSpeedX().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedX1 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedX(speedX1);

        // check
        final Speed speedX2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedX(speedX2);
        final Speed speedX3 = frame.getSpeedX();

        assertEquals(speedX1, speedX2);
        assertEquals(speedX1, speedX3);
    }

    @Test
    public void testGetSetSpeedY() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getSpeedY().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedY1 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedY(speedY1);

        // check
        final Speed speedY2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedY(speedY2);
        final Speed speedY3 = frame.getSpeedY();

        assertEquals(speedY1, speedY2);
        assertEquals(speedY1, speedY3);
    }

    @Test
    public void testGetSetSpeedZ() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        assertEquals(frame.getSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedZ1 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedZ(speedZ1);

        // check
        final Speed speedZ2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedZ(speedZ2);
        final Speed speedZ3 = frame.getSpeedZ();

        assertEquals(speedZ1, speedZ2);
        assertEquals(speedZ1, speedZ3);
    }

    @Test
    public void testSetSpeedCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final ECEFFrame frame = new ECEFFrame();

        // set new values
        frame.setSpeedCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(frame.getSpeedX(), speedX);
        assertEquals(frame.getSpeedY(), speedY);
        assertEquals(frame.getSpeedZ(), speedZ);
    }

    @Test
    public void testGetSetCoordinateTransformation() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final ECEFFrame frame = new ECEFFrame();

        // check initial value
        final CoordinateTransformation c1 = frame.getCoordinateTransformation();
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
        final CoordinateTransformation c2 = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        frame.setCoordinateTransformation(c2);

        // check
        assertEquals(frame.getCoordinateTransformation(), c2);
        final CoordinateTransformation c3 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        frame.getCoordinateTransformation(c3);
        assertEquals(c2, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            frame.setCoordinateTransformation(new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) { }
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m1 = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                m1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final ECEFFrame frame = new ECEFFrame(c);

        // check
        assertEquals(frame.getCoordinateTransformationMatrix(), m1);
        final Matrix m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m2, m1);
    }

    @Test
    public void testIsValidCoordinateTransformation() {

        final CoordinateTransformation c1 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final CoordinateTransformation c2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final CoordinateTransformation c3 = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(ECEFFrame.isValidCoordinateTransformation(c1));
        assertFalse(ECEFFrame.isValidCoordinateTransformation(c2));
        assertFalse(ECEFFrame.isValidCoordinateTransformation(c3));
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
        final CoordinateTransformation c = new CoordinateTransformation(
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
        assertEquals(frame1.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
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
        final CoordinateTransformation c = new CoordinateTransformation(
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
        assertEquals(frame1.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
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
        final CoordinateTransformation c = new CoordinateTransformation(
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
        final CoordinateTransformation c = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame2 = new ECEFFrame(x, y, z, vx, vy, vz, c);
        final ECEFFrame frame3 = new ECEFFrame();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frame1.equals((Object)frame1));
        assertTrue(frame1.equals(frame1));
        assertTrue(frame1.equals(frame2));
        assertFalse(frame1.equals(frame3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(frame1.equals((Object)null));
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
        final CoordinateTransformation c = new CoordinateTransformation(
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
        final CoordinateTransformation c = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final ECEFFrame frame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

        final Object frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }
}
