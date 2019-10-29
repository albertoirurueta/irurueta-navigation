package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GravityTest {

    private static final double MIN_VALUE = 9.80;
    private static final double MAX_VALUE = 9.82;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        Gravity gravity = new Gravity();

        // check default values
        assertEquals(gravity.getGx(), 0.0, 0.0);
        assertEquals(gravity.getGy(), 0.0, 0.0);
        assertEquals(gravity.getGz(), 0.0, 0.0);
        assertEquals(gravity.getGravityX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGravityY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getNorm(), 0.0, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with gravity coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        gravity = new Gravity(gx, gy, gz);

        // check default values
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);
        assertEquals(gravity.getGravityX().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGravityY().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravity.getNorm(), g, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor with acceleration coordinates
        final Acceleration gravityX = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityY = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityZ = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity = new Gravity(gravityX, gravityY, gravityZ);

        // check default values
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);
        assertEquals(gravity.getGravityX().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGravityY().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravity.getNorm(), g, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor from another gravity
        final Gravity gravity2 = new Gravity(gravity);

        // check default values
        assertEquals(gravity2.getGx(), gx, 0.0);
        assertEquals(gravity2.getGy(), gy, 0.0);
        assertEquals(gravity2.getGz(), gz, 0.0);
    }

    @Test
    public void testGetSetGx() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setGx(gx);

        // check
        assertEquals(gravity.getGx(), gx, 0.0);
    }

    @Test
    public void testGetSetGy() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setGy(gy);

        // check
        assertEquals(gravity.getGy(), gy, 0.0);
    }

    @Test
    public void testGetSetGz() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setGz(gz);

        // check
        assertEquals(gravity.getGz(), gz, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final Gravity gravity = new Gravity();

        // check default values
        assertEquals(gravity.getGx(), 0.0, 0.0);
        assertEquals(gravity.getGy(), 0.0, 0.0);
        assertEquals(gravity.getGz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setCoordinates(gx, gy, gz);

        // check
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);
    }

    @Test
    public void testGetSetGravityX() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGravityX().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityX1 = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGravityX(gravityX1);

        // check
        assertEquals(gravity.getGravityX().getValue().doubleValue(), gx, 0.0);

        final Acceleration gravityX2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGravityX(gravityX2);
        assertEquals(gravityX2.getValue().doubleValue(), gx, 0.0);
    }

    @Test
    public void testGetSetGravityY() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGravityY().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityY1 = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGravityY(gravityY1);

        // check
        assertEquals(gravity.getGravityY().getValue().doubleValue(), gy, 0.0);

        final Acceleration gravityY2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGravityY(gravityY2);
        assertEquals(gravityY2.getValue().doubleValue(), gy, 0.0);
    }

    @Test
    public void testGetSetGravityZ() {
        final Gravity gravity = new Gravity();

        // check default value
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityZ1 = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGravityZ(gravityZ1);

        // check
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), gz, 0.0);

        final Acceleration gravityZ2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGravityZ(gravityZ2);
        assertEquals(gravityZ2.getValue().doubleValue(), gz, 0.0);
    }

    @Test
    public void testSetGravityCoordinates() {
        final Gravity gravity = new Gravity();

        // check default values
        assertEquals(gravity.getGravityX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGravityY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityX1 = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityY1 = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityZ1 = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGravityCoordinates(gravityX1, gravityY1, gravityZ1);

        // check
        assertEquals(gravity.getGravityX().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGravityY().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGravityZ().getValue().doubleValue(), gz, 0.0);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        final Gravity gravity = new Gravity(gx, gy, gz);

        assertEquals(gravity.getNorm(), g, 0.0);

        Acceleration norm1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravity.getNormAsAcceleration(norm1);
        Acceleration norm2 = gravity.getNormAsAcceleration();

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

        final Gravity gravity1 = new Gravity(gx, gy, gz);
        final Gravity gravity2 = new Gravity();

        gravity1.copyTo(gravity2);

        // check
        assertEquals(gravity1.getGx(), gravity2.getGx(), 0.0);
        assertEquals(gravity1.getGy(), gravity2.getGy(), 0.0);
        assertEquals(gravity1.getGz(), gravity2.getGz(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity1 = new Gravity(gx, gy, gz);
        final Gravity gravity2 = new Gravity();

        gravity2.copyFrom(gravity1);

        // check
        assertEquals(gravity1.getGx(), gravity2.getGx(), 0.0);
        assertEquals(gravity1.getGy(), gravity2.getGy(), 0.0);
        assertEquals(gravity1.getGz(), gravity2.getGz(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity = new Gravity(gx, gy, gz);

        final double[] array1 = gravity.asArray();
        final double[] array2 = new double[Gravity.COMPONENTS];
        gravity.asArray(array2);

        // check
        assertEquals(array1[0], gx, 0.0);
        assertEquals(array1[1], gy, 0.0);
        assertEquals(array1[2], gz, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            gravity.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity = new Gravity(gx, gy, gz);

        final Matrix matrix1 = gravity.asMatrix();
        final Matrix matrix2 = new Matrix(1, 1);
        gravity.asMatrix(matrix2);
        final Matrix matrix3 = new Matrix(Gravity.COMPONENTS, 1);
        gravity.asMatrix(matrix3);

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

        final Gravity gravity1 = new Gravity(gx, gy, gz);
        final Gravity gravity2 = new Gravity(gx, gy, gz);
        final Gravity gravity3 = new Gravity();

        assertEquals(gravity1.hashCode(), gravity2.hashCode());
        assertNotEquals(gravity1.hashCode(), gravity3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity1 = new Gravity(gx, gy, gz);
        final Gravity gravity2 = new Gravity(gx, gy, gz);
        final Gravity gravity3 = new Gravity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(gravity1.equals((Object)gravity1));
        assertTrue(gravity1.equals(gravity1));
        assertTrue(gravity1.equals(gravity2));
        assertFalse(gravity1.equals(gravity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(gravity1.equals((Object)null));
        assertFalse(gravity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(gravity1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity1 = new Gravity(gx, gy, gz);
        final Gravity gravity2 = new Gravity(gx, gy, gz);
        final Gravity gravity3 = new Gravity();

        assertTrue(gravity1.equals(gravity1, THRESHOLD));
        assertTrue(gravity1.equals(gravity2, THRESHOLD));
        assertFalse(gravity1.equals(gravity3, THRESHOLD));
        assertFalse(gravity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Gravity gravity1 = new Gravity(gx, gy, gz);

        final Object gravity2 = gravity1.clone();

        // check
        assertEquals(gravity1, gravity2);
    }
}
