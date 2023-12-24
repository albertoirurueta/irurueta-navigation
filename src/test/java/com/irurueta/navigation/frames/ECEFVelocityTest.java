package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.io.IOException;
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
        assertEquals(0.0, velocity.getVx(), 0.0);
        assertEquals(0.0, velocity.getVy(), 0.0);
        assertEquals(0.0, velocity.getVz(), 0.0);

        Speed speedX = velocity.getSpeedX();
        Speed speedY = velocity.getSpeedY();
        Speed speedZ = velocity.getSpeedZ();

        assertEquals(0.0, speedX.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX.getUnit());
        assertEquals(0.0, speedY.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY.getUnit());
        assertEquals(0.0, speedZ.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ.getUnit());

        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        velocity = new ECEFVelocity(vx, vy, vz);

        // check
        assertEquals(vx, velocity.getVx(), 0.0);
        assertEquals(vy, velocity.getVy(), 0.0);
        assertEquals(vz, velocity.getVz(), 0.0);

        speedX = velocity.getSpeedX();
        speedY = velocity.getSpeedY();
        speedZ = velocity.getSpeedZ();

        assertEquals(vx, speedX.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX.getUnit());
        assertEquals(vy, speedY.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY.getUnit());
        assertEquals(vz, speedZ.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ.getUnit());


        // test constructor with coordinate distances
        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        velocity = new ECEFVelocity(speedX2, speedY2, speedZ2);

        // check
        assertEquals(vx, velocity.getVx(), 0.0);
        assertEquals(vy, velocity.getVy(), 0.0);
        assertEquals(vz, velocity.getVz(), 0.0);

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
        assertEquals(0.0, velocity.getVx(), 0.0);

        // set new value
        velocity.setVx(vx);

        // check
        assertEquals(vx, velocity.getVx(), 0.0);
    }

    @Test
    public void testGetSetVy() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        assertEquals(0.0, velocity.getVy(), 0.0);

        // set new value
        velocity.setVy(vy);

        // check
        assertEquals(vy, velocity.getVy(), 0.0);
    }

    @Test
    public void testGetSetVz() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        assertEquals(0.0, velocity.getVz(), 0.0);

        // set new value
        velocity.setVz(vz);

        // check
        assertEquals(vz, velocity.getVz(), 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default values
        assertEquals(0.0, velocity.getVx(), 0.0);
        assertEquals(0.0, velocity.getVy(), 0.0);
        assertEquals(0.0, velocity.getVz(), 0.0);

        // set new values
        velocity.setCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, velocity.getVx(), 0.0);
        assertEquals(vy, velocity.getVy(), 0.0);
        assertEquals(vz, velocity.getVz(), 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity();

        // check default value
        final Speed speedX1 = velocity.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

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

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

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

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

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
        assertEquals(speedX, velocity.getSpeedX());
        assertEquals(speedY, velocity.getSpeedY());
        assertEquals(speedZ, velocity.getSpeedZ());
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);

        final double norm = Math.sqrt(Math.pow(vx, 2.0) + Math.pow(vy, 2.0) + Math.pow(vz, 2.0));
        assertEquals(velocity.getNorm(), norm, ABSOLUTE_ERROR);

        final Speed normSpeed1 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getNormAsSpeed(normSpeed1);
        final Speed normSpeed2 = velocity.getNormAsSpeed();

        assertEquals(norm, normSpeed1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(SpeedUnit.METERS_PER_SECOND, normSpeed1.getUnit());
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
        assertEquals(vx, velocity2.getVx(), 0.0);
        assertEquals(vy, velocity2.getVy(), 0.0);
        assertEquals(vz, velocity2.getVz(), 0.0);
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
        assertEquals(vx, velocity2.getVx(), 0.0);
        assertEquals(vy, velocity2.getVy(), 0.0);
        assertEquals(vz, velocity2.getVz(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);

        final double[] result1 = new double[ECEFVelocity.COMPONENTS];
        velocity.asArray(result1);

        final double[] result2 = velocity.asArray();

        assertEquals(vx, result1[0], 0.0);
        assertEquals(vy, result1[1], 0.0);
        assertEquals(vz, result1[2], 0.0);

        assertArrayEquals(result1, result2, 0.0);

        // Force IllegalArgumentException
        try {
            velocity.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);

        final Matrix result1 = new Matrix(ECEFVelocity.COMPONENTS, 1);
        velocity.asMatrix(result1);

        final Matrix result2 = velocity.asMatrix();

        final Matrix result3 = new Matrix(1, 1);
        velocity.asMatrix(result3);

        assertEquals(vx, result1.getElementAtIndex(0), 0.0);
        assertEquals(vy, result1.getElementAtIndex(1), 0.0);
        assertEquals(vz, result1.getElementAtIndex(2), 0.0);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
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
        assertTrue(velocity1.equals((Object) velocity1));
        //noinspection EqualsWithItself
        assertTrue(velocity1.equals(velocity1));
        assertTrue(velocity1.equals(velocity2));
        assertFalse(velocity1.equals(velocity3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(velocity1.equals((Object) null));
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
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);

        final Object velocity2 = velocity1.clone();

        assertEquals(velocity1, velocity2);
    }
    
    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final ECEFVelocity velocity1 = new ECEFVelocity(vx, vy, vz);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(velocity1);
        final ECEFVelocity velocity2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(velocity1, velocity2);
        assertNotSame(velocity1, velocity2);
    }
}
