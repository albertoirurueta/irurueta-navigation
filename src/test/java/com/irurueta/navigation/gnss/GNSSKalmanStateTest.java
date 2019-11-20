package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GNSSKalmanStateTest {

    private static final double MIN_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final double MIN_CLOCK_OFFSET = -1e-5;
    private static final double MAX_CLOCK_OFFSET = 1e-5;

    private static final double MIN_CLOCK_DRIFT = -1e-6;
    private static final double MAX_CLOCK_DRIFT = 1e-6;

    private static final double THRESHOLD = 1e-8;

    @Test
    public void testConstructor() {
        // test empty constructor
        GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        assertEquals(state.getX(), 0.0, 0.0);
        assertEquals(state.getY(), 0.0, 0.0);
        assertEquals(state.getZ(), 0.0, 0.0);
        assertEquals(state.getVx(), 0.0, 0.0);
        assertEquals(state.getVy(), 0.0, 0.0);
        assertEquals(state.getVz(), 0.0, 0.0);
        assertEquals(state.getClockOffset(), 0.0, 0.0);
        assertEquals(state.getClockDrift(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        state = new GNSSKalmanState(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        // check default values
        assertEquals(state.getX(), x, 0.0);
        assertEquals(state.getY(), y, 0.0);
        assertEquals(state.getZ(), z, 0.0);
        assertEquals(state.getVx(), vx, 0.0);
        assertEquals(state.getVy(), vy, 0.0);
        assertEquals(state.getVz(), vz, 0.0);
        assertEquals(state.getClockOffset(), clockOffset, 0.0);
        assertEquals(state.getClockDrift(), clockDrift, 0.0);


        // test constructor with measurement values
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Distance clockOffsetDistance = new Distance(clockOffset,
                DistanceUnit.METER);
        final Speed clockDriftSpeed = new Speed(clockDrift,
                SpeedUnit.METERS_PER_SECOND);

        state = new GNSSKalmanState(distanceX, distanceY, distanceZ,
                speedX, speedY, speedZ, clockOffsetDistance, clockDriftSpeed);

        // check default values
        assertEquals(state.getX(), x, 0.0);
        assertEquals(state.getY(), y, 0.0);
        assertEquals(state.getZ(), z, 0.0);
        assertEquals(state.getVx(), vx, 0.0);
        assertEquals(state.getVy(), vy, 0.0);
        assertEquals(state.getVz(), vz, 0.0);
        assertEquals(state.getClockOffset(), clockOffset, 0.0);
        assertEquals(state.getClockDrift(), clockDrift, 0.0);


        // test constructor with measurement values and position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        state = new GNSSKalmanState(position, speedX, speedY, speedZ,
                clockOffsetDistance, clockDriftSpeed);

        // check default values
        assertEquals(state.getX(), x, 0.0);
        assertEquals(state.getY(), y, 0.0);
        assertEquals(state.getZ(), z, 0.0);
        assertEquals(state.getVx(), vx, 0.0);
        assertEquals(state.getVy(), vy, 0.0);
        assertEquals(state.getVz(), vz, 0.0);
        assertEquals(state.getClockOffset(), clockOffset, 0.0);
        assertEquals(state.getClockDrift(), clockDrift, 0.0);


        // test copy constructor
        final GNSSKalmanState state2 = new GNSSKalmanState(state);

        // check default values
        assertEquals(state2.getX(), x, 0.0);
        assertEquals(state2.getY(), y, 0.0);
        assertEquals(state2.getZ(), z, 0.0);
        assertEquals(state2.getVx(), vx, 0.0);
        assertEquals(state2.getVy(), vy, 0.0);
        assertEquals(state2.getVz(), vz, 0.0);
        assertEquals(state2.getClockOffset(), clockOffset, 0.0);
        assertEquals(state2.getClockDrift(), clockDrift, 0.0);
    }

    @Test
    public void testGetSetX() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        state.setX(x);

        // check
        assertEquals(state.getX(), x, 0.0);
    }

    @Test
    public void testGetSetDistanceX() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Distance distanceX1 = state.getDistanceX();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);

        state.setDistanceX(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        state.getDistanceX(distanceX3);
        final Distance distanceX4 = state.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetY() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        state.setY(y);

        // check
        assertEquals(state.getY(), y, 0.0);
    }

    @Test
    public void testGetSetDistanceY() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Distance distanceY1 = state.getDistanceY();

        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);

        state.setDistanceY(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        state.getDistanceY(distanceY3);
        final Distance distanceY4 = state.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetZ() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        state.setZ(z);

        // check
        assertEquals(state.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetDistanceZ() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Distance distanceZ1 = state.getDistanceZ();

        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        state.setDistanceZ(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        state.getDistanceZ(distanceZ3);
        final Distance distanceZ4 = state.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionCoordinates() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        assertEquals(state.getX(), 0.0, 0.0);
        assertEquals(state.getY(), 0.0, 0.0);
        assertEquals(state.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        state.setPositionCoordinates(x, y, z);

        // check
        assertEquals(state.getX(), x, 0.0);
        assertEquals(state.getY(), y, 0.0);
        assertEquals(state.getZ(), z, 0.0);
    }

    @Test
    public void testSetPositionCoordinatesWithDistances() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        final Distance distanceX1 = state.getDistanceX();
        final Distance distanceY1 = state.getDistanceY();
        final Distance distanceZ1 = state.getDistanceZ();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);
        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);
        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        state.setPositionCoordinates(distanceX2, distanceY2, distanceZ2);

        // check
        final Distance distanceX3 = state.getDistanceX();
        final Distance distanceY3 = state.getDistanceY();
        final Distance distanceZ3 = state.getDistanceZ();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceZ2, distanceZ3);
    }

    @Test
    public void testGetSetPosition() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Point3D position1 = state.getPosition();

        assertEquals(position1, Point3D.create());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);

        state.setPosition(position2);

        // check
        final Point3D position3 = Point3D.create();
        state.getPosition(position3);
        final Point3D position4 = state.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetVx() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getVx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        state.setVx(vx);

        // check
        assertEquals(state.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Speed speedX1 = state.getSpeedX();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedX(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedX(speedX3);
        final Speed speedX4 = state.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetVy() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getVy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        state.setVy(vy);

        // check
        assertEquals(state.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetSpeedY() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Speed speedY1 = state.getSpeedY();

        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedY(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedY(speedY3);
        final Speed speedY4 = state.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetVz() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        state.setVz(vz);

        // chekc
        assertEquals(state.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetSpeedZ() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Speed speedZ1 = state.getSpeedZ();

        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizser = new UniformRandomizer(new Random());
        final double vz = randomizser.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedZ(speedZ3);
        final Speed speedZ4 = state.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        assertEquals(state.getVx(), 0.0, 0.0);
        assertEquals(state.getVy(), 0.0, 0.0);
        assertEquals(state.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        state.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(state.getVx(), vx, 0.0);
        assertEquals(state.getVy(), vy, 0.0);
        assertEquals(state.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinatesWithSpeed() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        final Speed speedX1 = state.getSpeedX();
        final Speed speedY1 = state.getSpeedY();
        final Speed speedZ1 = state.getSpeedZ();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setVelocityCoordinates(speedX2, speedY2, speedZ2);

        // check
        final Speed speedX3 = state.getSpeedX();
        final Speed speedY3 = state.getSpeedY();
        final Speed speedZ3 = state.getSpeedZ();

        assertEquals(speedX2, speedX3);
        assertEquals(speedY2, speedY3);
        assertEquals(speedZ2, speedZ3);
    }

    @Test
    public void testGetSetClockOffset() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getClockOffset(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);

        state.setClockOffset(clockOffset);

        // check
        assertEquals(state.getClockOffset(), clockOffset, 0.0);
    }

    @Test
    public void testGetSetClockOffsetDistance() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Distance clockOffset1 = state.getClockOffsetDistance();

        assertEquals(clockOffset1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(clockOffset1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);

        final Distance clockOffset2 = new Distance(clockOffset, DistanceUnit.METER);

        state.setClockOffset(clockOffset2);

        // check
        final Distance clockOffset3 = new Distance(0.0, DistanceUnit.KILOMETER);
        state.getClockOffsetDistance(clockOffset3);
        final Distance clockOffset4 = state.getClockOffsetDistance();

        assertEquals(clockOffset2, clockOffset3);
        assertEquals(clockOffset2, clockOffset4);
    }

    @Test
    public void testGetSetClockDrift() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertEquals(state.getClockDrift(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        state.setClockDrift(clockDrift);

        // check
        assertEquals(state.getClockDrift(), clockDrift, 0.0);
    }

    @Test
    public void testGetSetClockDriftSpeed() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        final Speed clockDrift1 = state.getClockDriftSpeed();

        assertEquals(clockDrift1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(clockDrift1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new vlaue
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final Speed clockDrift2 = new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);

        state.setClockDrift(clockDrift2);

        // check
        final Speed clockDrift3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getClockDriftSpeed(clockDrift3);
        final Speed clockDrift4 = state.getClockDriftSpeed();

        assertEquals(clockDrift2, clockDrift3);
        assertEquals(clockDrift2, clockDrift4);
    }

    @Test
    public void testCopyto() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);

        final GNSSKalmanState state2 = new GNSSKalmanState();
        state1.copyTo(state2);

        // check
        assertEquals(state2.getX(), x, 0.0);
        assertEquals(state2.getY(), y, 0.0);
        assertEquals(state2.getZ(), z, 0.0);
        assertEquals(state2.getVx(), vx, 0.0);
        assertEquals(state2.getVy(), vy, 0.0);
        assertEquals(state2.getVz(), vz, 0.0);
        assertEquals(state2.getClockOffset(), clockOffset, 0.0);
        assertEquals(state2.getClockDrift(), clockDrift, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);

        final GNSSKalmanState state2 = new GNSSKalmanState();
        state2.copyFrom(state1);

        // check
        assertEquals(state2.getX(), x, 0.0);
        assertEquals(state2.getY(), y, 0.0);
        assertEquals(state2.getZ(), z, 0.0);
        assertEquals(state2.getVx(), vx, 0.0);
        assertEquals(state2.getVy(), vy, 0.0);
        assertEquals(state2.getVz(), vz, 0.0);
        assertEquals(state2.getClockOffset(), clockOffset, 0.0);
        assertEquals(state2.getClockDrift(), clockDrift, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state2 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        assertEquals(state1.hashCode(), state2.hashCode());
        assertNotEquals(state1.hashCode(), state3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state2 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(state1.equals((Object)state1));
        assertTrue(state1.equals(state1));
        assertTrue(state1.equals(state2));
        assertFalse(state1.equals(state3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(state1.equals((Object)null));
        assertFalse(state1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(state1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state2 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        assertTrue(state1.equals(state1, THRESHOLD));
        assertTrue(state1.equals(state2, THRESHOLD));
        assertFalse(state1.equals(state3, THRESHOLD));
        assertFalse(state1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSKalmanState state1 = new GNSSKalmanState(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);

        final Object state2 = state1.clone();

        assertEquals(state1, state2);
    }
}
