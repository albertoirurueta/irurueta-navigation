package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GravitationECITest {

    private static final double MIN_VALUE = 9.80;
    private static final double MAX_VALUE = 9.82;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        GravitationECI gravitation = new GravitationECI();

        // check default values
        assertEquals(gravitation.getGx(), 0.0, 0.0);
        assertEquals(gravitation.getGy(), 0.0, 0.0);
        assertEquals(gravitation.getGz(), 0.0, 0.0);
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravitation.getNorm(), 0.0, 0.0);
        assertEquals(gravitation.getNormAsAcceleration().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with gravitation coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        gravitation = new GravitationECI(gx, gy, gz);

        // check default values
        assertEquals(gravitation.getGx(), gx, 0.0);
        assertEquals(gravitation.getGy(), gy, 0.0);
        assertEquals(gravitation.getGz(), gz, 0.0);
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravitation.getNorm(), g, 0.0);
        assertEquals(gravitation.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor with acceleration coordinates
        final Acceleration gravitationX = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravitationY = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravitationZ = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation = new GravitationECI(gravitationX, gravitationY, gravitationZ);

        // check default values
        assertEquals(gravitation.getGx(), gx, 0.0);
        assertEquals(gravitation.getGy(), gy, 0.0);
        assertEquals(gravitation.getGz(), gz, 0.0);
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravitation.getNorm(), g, 0.0);
        assertEquals(gravitation.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor from another gravitation
        final GravitationECI gravitation2 = new GravitationECI(gravitation);

        // check default values
        assertEquals(gravitation2.getGx(), gx, 0.0);
        assertEquals(gravitation2.getGy(), gy, 0.0);
        assertEquals(gravitation2.getGz(), gz, 0.0);
        assertEquals(gravitation2.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravitation2.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravitation2.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravitation2.getNorm(), g, 0.0);
        assertEquals(gravitation2.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);
    }

    @Test
    public void testGetSetGx() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGx(gx);

        // check
        assertEquals(gravitation.getGx(), gx, 0.0);
    }

    @Test
    public void testGetSetGy() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGy(gy);

        // check
        assertEquals(gravitation.getGy(), gy, 0.0);
    }

    @Test
    public void testGetSetGz() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGz(gz);

        // check
        assertEquals(gravitation.getGz(), gz, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final GravitationECI gravitation = new GravitationECI();

        // check default values
        assertEquals(gravitation.getGx(), 0.0, 0.0);
        assertEquals(gravitation.getGy(), 0.0, 0.0);
        assertEquals(gravitation.getGz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setCoordinates(gx, gy, gz);

        // check
        assertEquals(gravitation.getGx(), gx, 0.0);
        assertEquals(gravitation.getGy(), gy, 0.0);
        assertEquals(gravitation.getGz(), gz, 0.0);
    }

    @Test
    public void testGetSetGxAsAcceleration() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravitationX1 = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGx(gravitationX1);

        // check
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);

        final Acceleration gravitationX2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGxAsAcceleration(gravitationX2);
        final Acceleration gravitationX3 = gravitation.getGxAsAcceleration();
        assertEquals(gravitationX2.getValue().doubleValue(), gx, 0.0);
        assertEquals(gravitationX2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravitationX1, gravitationX2);
        assertEquals(gravitationX1, gravitationX3);
    }

    @Test
    public void testGetSetGyAsAcceleration() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);


        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravitationY1 = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGy(gravitationY1);

        // check
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);

        final Acceleration gravitationY2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGyAsAcceleration(gravitationY2);
        final Acceleration gravitationY3 = gravitation.getGyAsAcceleration();
        assertEquals(gravitationY2.getValue().doubleValue(), gy, 0.0);
        assertEquals(gravitationY2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravitationY1, gravitationY2);
        assertEquals(gravitationY1, gravitationY3);
    }

    @Test
    public void testGetSetGzAsAcceleration() {
        final GravitationECI gravitation = new GravitationECI();

        // check default value
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravitationZ1 = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGz(gravitationZ1);

        // check
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);

        final Acceleration gravitationZ2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGzAsAcceleration(gravitationZ2);
        final Acceleration gravitationZ3 = gravitation.getGzAsAcceleration();
        assertEquals(gravitationZ2.getValue().doubleValue(), gz, 0.0);
        assertEquals(gravitationZ2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravitationZ1, gravitationZ2);
        assertEquals(gravitationZ1, gravitationZ3);
    }

    @Test
    public void testSetCoordinatesFromAccelerations() {
        final GravitationECI gravitation = new GravitationECI();

        // check default values
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravitationX1 = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravitationY1 = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravitationZ1 = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setCoordinates(gravitationX1, gravitationY1, gravitationZ1);

        // check
        assertEquals(gravitation.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravitation.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravitation.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        final GravitationECI gravitation = new GravitationECI(gx, gy, gz);

        assertEquals(gravitation.getNorm(), g, 0.0);

        final Acceleration norm1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravitation.getNormAsAcceleration(norm1);
        final Acceleration norm2 = gravitation.getNormAsAcceleration();

        assertEquals(norm1.getValue().doubleValue(), g, 0.0);
        assertEquals(norm1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(norm1, norm2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation2 = new GravitationECI();

        gravitation1.copyTo(gravitation2);

        // check
        assertEquals(gravitation1.getGx(), gravitation2.getGx(), 0.0);
        assertEquals(gravitation1.getGy(), gravitation2.getGy(), 0.0);
        assertEquals(gravitation1.getGz(), gravitation2.getGz(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation2 = new GravitationECI();

        gravitation2.copyFrom(gravitation1);

        // check
        assertEquals(gravitation1.getGx(), gravitation2.getGx(), 0.0);
        assertEquals(gravitation1.getGy(), gravitation2.getGy(), 0.0);
        assertEquals(gravitation1.getGz(), gravitation2.getGz(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation = new GravitationECI(gx, gy, gz);

        final double[] array1 = gravitation.asArray();
        final double[] array2 = new double[GravitationECI.COMPONENTS];
        gravitation.asArray(array2);

        // check
        assertEquals(array1[0], gx, 0.0);
        assertEquals(array1[1], gy, 0.0);
        assertEquals(array2[2], gz, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            gravitation.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation = new GravitationECI(gx, gy, gz);

        final Matrix matrix1 = gravitation.asMatrix();
        final Matrix matrix2 = new Matrix(1, 1);
        gravitation.asMatrix(matrix2);
        final Matrix matrix3 = new Matrix(GravitationECI.COMPONENTS, 1);
        gravitation.asMatrix(matrix3);

        // check
        assertEquals(matrix1.getElementAtIndex(0), gx, 0.0);
        assertEquals(matrix1.getElementAtIndex(1), gy, 0.0);
        assertEquals(matrix1.getElementAtIndex(2), gz, 0.0);
        assertEquals(matrix1, matrix2);
        assertEquals(matrix1, matrix3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation2 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation3 = new GravitationECI();

        assertEquals(gravitation1.hashCode(), gravitation2.hashCode());
        assertNotEquals(gravitation1.hashCode(), gravitation3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation2 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation3 = new GravitationECI();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(gravitation1.equals((Object)gravitation1));
        assertTrue(gravitation1.equals(gravitation1));
        assertTrue(gravitation1.equals(gravitation2));
        assertFalse(gravitation1.equals(gravitation3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(gravitation1.equals((Object)null));
        assertFalse(gravitation1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(gravitation1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation2 = new GravitationECI(gx, gy, gz);
        final GravitationECI gravitation3 = new GravitationECI();

        assertTrue(gravitation1.equals(gravitation1, THRESHOLD));
        assertTrue(gravitation1.equals(gravitation2, THRESHOLD));
        assertFalse(gravitation1.equals(gravitation3, THRESHOLD));
        assertFalse(gravitation1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GravitationECI gravitation1 = new GravitationECI(gx, gy, gz);

        final Object gravitation2 = gravitation1.clone();

        // check
        assertEquals(gravitation1, gravitation2);
    }
}
