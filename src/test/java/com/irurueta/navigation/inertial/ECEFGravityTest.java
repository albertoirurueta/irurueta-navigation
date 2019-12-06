/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECEFGravityTest {

    private static final double MIN_VALUE = 9.80;
    private static final double MAX_VALUE = 9.82;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        ECEFGravity gravity = new ECEFGravity();

        // check default values
        assertEquals(gravity.getGx(), 0.0, 0.0);
        assertEquals(gravity.getGy(), 0.0, 0.0);
        assertEquals(gravity.getGz(), 0.0, 0.0);
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getNorm(), 0.0, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with gravity coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        gravity = new ECEFGravity(gx, gy, gz);

        // check default values
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravity.getNorm(), g, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor with acceleration coordinates
        final Acceleration gravityX = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityY = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityZ = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity = new ECEFGravity(gravityX, gravityY, gravityZ);

        // check default values
        assertEquals(gravity.getGx(), gx, 0.0);
        assertEquals(gravity.getGy(), gy, 0.0);
        assertEquals(gravity.getGz(), gz, 0.0);
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravity.getNorm(), g, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);


        // test constructor from another gravity
        final ECEFGravity gravity2 = new ECEFGravity(gravity);

        // check default values
        assertEquals(gravity2.getGx(), gx, 0.0);
        assertEquals(gravity2.getGy(), gy, 0.0);
        assertEquals(gravity2.getGz(), gz, 0.0);
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
        assertEquals(gravity.getNorm(), g, 0.0);
        assertEquals(gravity.getNormAsAcceleration().getValue().doubleValue(), g, 0.0);
    }

    @Test
    public void testGetSetGx() {
        final ECEFGravity gravity = new ECEFGravity();

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
        final ECEFGravity gravity = new ECEFGravity();

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
        final ECEFGravity gravity = new ECEFGravity();

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
        final ECEFGravity gravity = new ECEFGravity();

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
    public void testGetSetGxAsAcceleration() {
        final ECEFGravity gravity = new ECEFGravity();

        // check default value
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityX1 = new Acceleration(gx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGx(gravityX1);

        // check
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);

        final Acceleration gravityX2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGxAsAcceleration(gravityX2);
        final Acceleration gravityX3 = gravity.getGxAsAcceleration();
        assertEquals(gravityX2.getValue().doubleValue(), gx, 0.0);
        assertEquals(gravityX2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravityX1, gravityX2);
        assertEquals(gravityX1, gravityX3);
    }

    @Test
    public void testGetSetGyAsAcceleration() {
        final ECEFGravity gravity = new ECEFGravity();

        // check default value
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityY1 = new Acceleration(gy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGy(gravityY1);

        // check
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);

        final Acceleration gravityY2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGyAsAcceleration(gravityY2);
        final Acceleration gravityY3 = gravity.getGyAsAcceleration();
        assertEquals(gravityY2.getValue().doubleValue(), gy, 0.0);
        assertEquals(gravityY2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravityY1, gravityY2);
        assertEquals(gravityY1, gravityY3);
    }

    @Test
    public void testGetSetGzAsAcceleration() {
        final ECEFGravity gravity = new ECEFGravity();

        // check default value
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityZ1 = new Acceleration(gz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGz(gravityZ1);

        // check
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);

        final Acceleration gravityZ2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGzAsAcceleration(gravityZ2);
        final Acceleration gravityZ3 = gravity.getGzAsAcceleration();
        assertEquals(gravityZ2.getValue().doubleValue(), gz, 0.0);
        assertEquals(gravityZ2.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(gravityZ1, gravityZ2);
        assertEquals(gravityZ1, gravityZ3);
    }

    @Test
    public void testSetCoordinatesFromAccelerations() {
        final ECEFGravity gravity = new ECEFGravity();

        // check default values
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), 0.0, 0.0);

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

        gravity.setCoordinates(gravityX1, gravityY1, gravityZ1);

        // check
        assertEquals(gravity.getGxAsAcceleration().getValue().doubleValue(), gx, 0.0);
        assertEquals(gravity.getGyAsAcceleration().getValue().doubleValue(), gy, 0.0);
        assertEquals(gravity.getGzAsAcceleration().getValue().doubleValue(), gz, 0.0);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        final ECEFGravity gravity = new ECEFGravity(gx, gy, gz);

        assertEquals(gravity.getNorm(), g, 0.0);

        final Acceleration norm1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravity.getNormAsAcceleration(norm1);
        final Acceleration norm2 = gravity.getNormAsAcceleration();

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

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity2 = new ECEFGravity();

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

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity2 = new ECEFGravity();

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

        final ECEFGravity gravity = new ECEFGravity(gx, gy, gz);

        final double[] array1 = gravity.asArray();
        final double[] array2 = new double[ECEFGravity.COMPONENTS];
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

        final ECEFGravity gravity = new ECEFGravity(gx, gy, gz);

        final Matrix matrix1 = gravity.asMatrix();
        final Matrix matrix2 = new Matrix(1, 1);
        gravity.asMatrix(matrix2);
        final Matrix matrix3 = new Matrix(ECEFGravity.COMPONENTS, 1);
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

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity2 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity3 = new ECEFGravity();

        assertEquals(gravity1.hashCode(), gravity2.hashCode());
        assertNotEquals(gravity1.hashCode(), gravity3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity2 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity3 = new ECEFGravity();

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

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity2 = new ECEFGravity(gx, gy, gz);
        final ECEFGravity gravity3 = new ECEFGravity();

        assertTrue(gravity1.equals(gravity1, THRESHOLD));
        assertTrue(gravity1.equals(gravity2, THRESHOLD));
        assertFalse(gravity1.equals(gravity3, THRESHOLD));
        assertFalse(gravity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFGravity gravity1 = new ECEFGravity(gx, gy, gz);

        final Object gravity2 = gravity1.clone();

        // check
        assertEquals(gravity1, gravity2);
    }
}
