/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class NEDMagneticFluxDensityTest {

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.

    private static final double MIN_VALUE = 30e-6;
    private static final double MAX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);
        assertEquals(b.getNorm(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        b = new NEDMagneticFluxDensity(bn, be, bd);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);


        // test copy constructor
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(b);

        assertEquals(b2.getBn(), bn, 0.0);
        assertEquals(b2.getBe(), be, 0.0);
        assertEquals(b2.getBd(), bd, 0.0);
        assertEquals(b2.getNorm(), b.getNorm(),
                0.0);
    }

    @Test
    public void testGetSetBn() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBn(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBn(bn);

        // check
        assertEquals(b.getBn(), bn, 0.0);
    }

    @Test
    public void testGetSetBe() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBe(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBe(be);

        // check
        assertEquals(b.getBe(), be, 0.0);
    }

    @Test
    public void testGetSetBd() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBd(bd);

        // check
        assertEquals(b.getBd(), bd, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
    }

    @Test
    public void testGetNorm() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getNorm(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity();

        b1.copyTo(b2);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity();

        b2.copyFrom(b1);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity(
                bn, be, bd);

        final double[] array1 = b.asArray();
        final double[] array2 = new double[
                NEDMagneticFluxDensity.COMPONENTS];
        b.asArray(array2);

        // check
        assertEquals(array1[0], bn, 0.0);
        assertEquals(array1[1], be, 0.0);
        assertEquals(array1[2], bd, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            b.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity(
                bn, be, bd);

        final Matrix m1 = b.asMatrix();
        final Matrix m2 = new Matrix(1, 1);
        b.asMatrix(m2);
        final Matrix m3 = new Matrix(NEDMagneticFluxDensity.COMPONENTS,
                1);
        b.asMatrix(m3);

        // check
        assertEquals(m1.getElementAtIndex(0), bn, 0.0);
        assertEquals(m1.getElementAtIndex(1), be, 0.0);
        assertEquals(m1.getElementAtIndex(2), bd, 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(b1.equals((Object) b1));
        assertTrue(b1.equals(b1));
        assertTrue(b1.equals(b2));
        assertFalse(b1.equals(b3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(b1.equals((Object) null));
        assertFalse(b1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(b1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

        assertTrue(b1.equals(b1, THRESHOLD));
        assertTrue(b1.equals(b2, THRESHOLD));
        assertFalse(b1.equals(b3, THRESHOLD));
        assertFalse(b1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);

        final Object b2 = b1.clone();

        // check
        assertEquals(b1, b2);
    }
}
