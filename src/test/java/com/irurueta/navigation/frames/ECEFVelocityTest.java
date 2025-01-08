package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ECEFVelocityTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstructor() {

        // test empty constructor
        var velocity = new ECEFVelocity();

        // check
        assertEquals(0.0, velocity.getVx(), 0.0);
        assertEquals(0.0, velocity.getVy(), 0.0);
        assertEquals(0.0, velocity.getVz(), 0.0);

        var speedX = velocity.getSpeedX();
        var speedY = velocity.getSpeedY();
        var speedZ = velocity.getSpeedZ();

        assertEquals(0.0, speedX.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX.getUnit());
        assertEquals(0.0, speedY.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY.getUnit());
        assertEquals(0.0, speedZ.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ.getUnit());

        // test constructor with coordinates
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

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
        final var velocity2 = new ECEFVelocity(velocity);

        // check
        assertEquals(velocity.getVx(), velocity2.getVx(), 0.0);
        assertEquals(velocity.getVy(), velocity2.getVy(), 0.0);
        assertEquals(velocity.getVz(), velocity2.getVz(), 0.0);
        assertEquals(velocity, velocity2);
    }

    @Test
    void testGetSetVx() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        assertEquals(0.0, velocity.getVx(), 0.0);

        // set new value
        velocity.setVx(vx);

        // check
        assertEquals(vx, velocity.getVx(), 0.0);
    }

    @Test
    void testGetSetVy() {
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        assertEquals(0.0, velocity.getVy(), 0.0);

        // set new value
        velocity.setVy(vy);

        // check
        assertEquals(vy, velocity.getVy(), 0.0);
    }

    @Test
    void testGetSetVz() {
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        assertEquals(0.0, velocity.getVz(), 0.0);

        // set new value
        velocity.setVz(vz);

        // check
        assertEquals(vz, velocity.getVz(), 0.0);
    }

    @Test
    void testSetCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

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
    void testGetSetSpeedX() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        final var speedX1 = velocity.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        velocity.setVx(speedX2);

        // check
        final var speedX3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedX(speedX3);
        final var speedX4 = velocity.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    void testGetSetSpeedY() {
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        final var speedY1 = velocity.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        velocity.setVy(speedY2);

        // check
        final var speedY3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedY(speedY3);
        final var speedY4 = velocity.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    void testGetSetSpeedZ() {
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default value
        final var speedZ1 = velocity.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        velocity.setVz(speedZ2);

        // check
        final var speedZ3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getSpeedZ(speedZ3);
        final var speedZ4 = velocity.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    void testSetSpeedCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity();

        // check default values
        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        velocity.setCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(speedX, velocity.getSpeedX());
        assertEquals(speedY, velocity.getSpeedY());
        assertEquals(speedZ, velocity.getSpeedZ());
    }

    @Test
    void testGetNorm() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity(vx, vy, vz);

        final var norm = Math.sqrt(Math.pow(vx, 2.0) + Math.pow(vy, 2.0) + Math.pow(vz, 2.0));
        assertEquals(velocity.getNorm(), norm, ABSOLUTE_ERROR);

        final var normSpeed1 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        velocity.getNormAsSpeed(normSpeed1);
        final var normSpeed2 = velocity.getNormAsSpeed();

        assertEquals(norm, normSpeed1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(SpeedUnit.METERS_PER_SECOND, normSpeed1.getUnit());
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);
        final var velocity2 = new ECEFVelocity();

        velocity1.copyTo(velocity2);

        // check
        assertEquals(vx, velocity2.getVx(), 0.0);
        assertEquals(vy, velocity2.getVy(), 0.0);
        assertEquals(vz, velocity2.getVz(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);
        final var velocity2 = new ECEFVelocity();

        velocity2.copyFrom(velocity1);

        // check
        assertEquals(vx, velocity2.getVx(), 0.0);
        assertEquals(vy, velocity2.getVy(), 0.0);
        assertEquals(vz, velocity2.getVz(), 0.0);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity(vx, vy, vz);

        final var result1 = new double[ECEFVelocity.COMPONENTS];
        velocity.asArray(result1);

        final var result2 = velocity.asArray();

        assertEquals(vx, result1[0], 0.0);
        assertEquals(vy, result1[1], 0.0);
        assertEquals(vz, result1[2], 0.0);

        assertArrayEquals(result1, result2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> velocity.asArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new ECEFVelocity(vx, vy, vz);

        final var result1 = new Matrix(ECEFVelocity.COMPONENTS, 1);
        velocity.asMatrix(result1);

        final var result2 = velocity.asMatrix();

        final var result3 = new Matrix(1, 1);
        velocity.asMatrix(result3);

        assertEquals(vx, result1.getElementAtIndex(0), 0.0);
        assertEquals(vy, result1.getElementAtIndex(1), 0.0);
        assertEquals(vz, result1.getElementAtIndex(2), 0.0);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);
        final var velocity2 = new ECEFVelocity(vx, vy, vz);
        final var velocity3 = new ECEFVelocity();

        assertEquals(velocity1.hashCode(), velocity2.hashCode());
        assertNotEquals(velocity1.hashCode(), velocity3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);
        final var velocity2 = new ECEFVelocity(vx, vy, vz);
        final var velocity3 = new ECEFVelocity();

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
        assertNotEquals(new Object(), velocity1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);
        final var velocity2 = new ECEFVelocity(vx, vy, vz);
        final var velocity3 = new ECEFVelocity();

        assertTrue(velocity1.equals(velocity1, THRESHOLD));
        assertTrue(velocity1.equals(velocity2, THRESHOLD));
        assertFalse(velocity1.equals(velocity3, THRESHOLD));
        assertFalse(velocity1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);

        final var velocity2 = velocity1.clone();

        assertEquals(velocity1, velocity2);
    }
    
    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new ECEFVelocity(vx, vy, vz);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(velocity1);
        final var velocity2 = SerializationHelper.<ECEFVelocity>deserialize(bytes);

        // check
        assertEquals(velocity1, velocity2);
        assertNotSame(velocity1, velocity2);
    }
}
