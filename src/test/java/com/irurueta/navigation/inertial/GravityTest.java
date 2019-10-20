package com.irurueta.navigation.inertial;

import com.irurueta.statistics.UniformRandomizer;
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


        // test constructor with gravity coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity = new Gravity(gx, gy, gz);

        // check default values
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);


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
