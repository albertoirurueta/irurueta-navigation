package com.irurueta.navigation.inertial;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECEFVelocityTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructor() {

        // test empty constructor
        ECEFVelocity velocity = new ECEFVelocity();

        // check
        assertEquals(velocity.getVx(), 0.0, 0.0);
        assertEquals(velocity.getVy(), 0.0, 0.0);
        assertEquals(velocity.getVz(), 0.0, 0.0);

        Speed speedX = velocity.getSpeedX();
        Speed speedY = velocity.getSpeedY();
        Speed speedZ = velocity.getSpeedZ();

        assertEquals(speedX.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedY.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedZ.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        velocity = new ECEFVelocity(vx, vy, vz);

        // check
        assertEquals(velocity.getVx(), vx, 0.0);
        assertEquals(velocity.getVy(), vy, 0.0);
        assertEquals(velocity.getVz(), vz, 0.0);

        speedX = velocity.getSpeedX();
        speedY = velocity.getSpeedY();
        speedZ = velocity.getSpeedZ();

        assertEquals(speedX.getValue().doubleValue(), vx, 0.0);
        assertEquals(speedX.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedY.getValue().doubleValue(), vy, 0.0);
        assertEquals(speedY.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedZ.getValue().doubleValue(), vz, 0.0);
        assertEquals(speedZ.getUnit(), SpeedUnit.METERS_PER_SECOND);


        // test constructor with coordinate distances
        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        velocity = new ECEFVelocity(speedX2, speedY2, speedZ2);

        // check
        assertEquals(velocity.getVx(), vx, 0.0);
        assertEquals(velocity.getVy(), vy, 0.0);
        assertEquals(velocity.getVz(), vz, 0.0);

        speedX = velocity.getSpeedX();
        speedY = velocity.getSpeedY();
        speedZ = velocity.getSpeedZ();

        assertEquals(speedX, speedX2);
        assertEquals(speedY, speedY2);
        assertEquals(speedZ, speedZ2);


        // test copy constructor
        final ECEFVelocity velocity2 = new ECEFVelocity(velocity);

        // check
        assertEquals(velocity.getVx(), velocity2.getVx(), 0.0);
        assertEquals(velocity.getVy(), velocity2.getVy(), 0.0);
        assertEquals(velocity.getVz(), velocity2.getVz(), 0.0);
        assertEquals(velocity, velocity2);
    }

    @Test
    public void testGetSetVx() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        assertEquals(velocity.getVx(), 0.0, 0.0);

        // set new value
        velocity.setVx(vx);

        // check
        assertEquals(velocity.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetVy() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        assertEquals(velocity.getVy(), 0.0, 0.0);

        // set new value
        velocity.setVy(vy);

        // check
        assertEquals(velocity.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetVz() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        assertEquals(velocity.getVz(), 0.0, 0.0);

        // set new value
        velocity.setVz(vz);

        // check
        assertEquals(velocity.getVz(), vz, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default values
        assertEquals(velocity.getVx(), 0.0, 0.0);
        assertEquals(velocity.getVy(), 0.0, 0.0);
        assertEquals(velocity.getVz(), 0.0, 0.0);

        // set new values
        velocity.setCoordinates(vx, vy, vz);

        // check
        assertEquals(velocity.getVx(), vx, 0.0);
        assertEquals(velocity.getVy(), vy, 0.0);
        assertEquals(velocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        final Speed speedX1 = velocity.getSpeedX();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        velocity.setVx(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedX(speedX3);
        final Speed speedX4 = velocity.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetSpeedY() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        final Speed speedY1 = velocity.getSpeedY();

        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        velocity.setVy(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedY(speedY3);
        final Speed speedY4 = velocity.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        final Speed speedZ1 = velocity.getSpeedZ();

        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        velocity.setVz(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedZ(speedZ3);
        final Speed speedZ4 = velocity.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetSpeedCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default values
        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        velocity.setCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(velocity.getSpeedX(), speedX);
        assertEquals(velocity.getSpeedY(), speedY);
        assertEquals(velocity.getSpeedZ(), speedZ);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);

        final double norm = Math.sqrt(Math.pow(vx, 2.0) + Math.pow(vy, 2.0)
                + Math.pow(vz, 2.0));
        assertEquals(velocity.getNorm(), norm, ABSOLUTE_ERROR);

        final Speed normSpeed1 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getNormAsSpeed(normSpeed1);
        final Speed normSpeed2 = velocity.getNormAsSpeed();

        assertEquals(normSpeed1.getValue().doubleValue(), norm, ABSOLUTE_ERROR);
        assertEquals(normSpeed1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity2 = new ECEFVelocity();

        velocity1.copyTo(velocity2);

        // check
        assertEquals(velocity2.getVx(), vx, 0.0);
        assertEquals(velocity2.getVy(), vy, 0.0);
        assertEquals(velocity2.getVz(), vz, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity2 = new ECEFVelocity();

        velocity2.copyFrom(velocity1);

        // check
        assertEquals(velocity2.getVx(), vx, 0.0);
        assertEquals(velocity2.getVy(), vy, 0.0);
        assertEquals(velocity2.getVz(), vz, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity3 = new ECEFVelocity();

        assertEquals(velocity1.hashCode(), velocity2.hashCode());
        assertNotEquals(velocity1.hashCode(), velocity3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity3 = new ECEFVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(velocity1.equals((Object)velocity1));
        assertTrue(velocity1.equals(velocity1));
        assertTrue(velocity1.equals(velocity2));
        assertFalse(velocity1.equals(velocity3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(velocity1.equals((Object)null));
        assertFalse(velocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(velocity1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity velocity3 = new ECEFVelocity();

        assertTrue(velocity1.equals(velocity1, THRESHOLD));
        assertTrue(velocity1.equals(velocity2, THRESHOLD));
        assertFalse(velocity1.equals(velocity3, THRESHOLD));
        assertFalse(velocity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);

        final Object velocity2 = velocity1.clone();

        assertEquals(velocity1, velocity2);
    }
}
