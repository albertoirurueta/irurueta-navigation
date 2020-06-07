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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class StandardDeviationBodyMagneticFluxDensityTest {

    private static final double MIN_MAGNETIC_FLUX_DENSITY = -70e-6;
    private static final double MAX_MAGNETIC_FLUX_DENSITY = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        StandardDeviationBodyMagneticFluxDensity stdMagnetic =
                new StandardDeviationBodyMagneticFluxDensity();

        // check default values
        assertNull(stdMagnetic.getMagneticFluxDensity());
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                0.0, 0.0);


        // test constructor with magnetic flux density
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity();
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(
                magneticFluxDensity);

        // check default values
        assertSame(stdMagnetic.getMagneticFluxDensity(), magneticFluxDensity);
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                0.0, 0.0);


        // test constructor with standard deviation
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(std);

        // check default values
        assertNull(stdMagnetic.getMagneticFluxDensity());
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                std, 0.0);

        // Force IllegalArgumentException
        stdMagnetic = null;
        try {
            stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(
                    -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdMagnetic);


        // test constructor with magnetic flux density and standard deviation
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(
                magneticFluxDensity, std);

        // check default values
        assertSame(stdMagnetic.getMagneticFluxDensity(), magneticFluxDensity);
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                std, 0.0);

        // Force IllegalArgumentException
        stdMagnetic = null;
        try {
            stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(
                    magneticFluxDensity, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdMagnetic);


        // test copy constructor
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(
                magneticFluxDensity, std);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(stdMagnetic);

        // check
        assertEquals(stdMagnetic.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testGetSetMagneticFluxDensity() {
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic =
                new StandardDeviationBodyMagneticFluxDensity();

        // check default value
        assertNull(stdMagnetic.getMagneticFluxDensity());

        // set new value
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity();
        stdMagnetic.setMagneticFluxDensity(magneticFluxDensity);

        // check
        assertSame(stdMagnetic.getMagneticFluxDensity(),
                magneticFluxDensity);
    }

    @Test
    public void testGetSetMagneticFluxDensityStandardDeviation() {
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic =
                new StandardDeviationBodyMagneticFluxDensity();

        // check default value
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        stdMagnetic.setMagneticFluxDensityStandardDeviation(std);

        // check
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(),
                std, 0.0);

        // Force IllegalArgumentException
        try {
            stdMagnetic.setMagneticFluxDensityStandardDeviation(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtSourceandDestinationIsEmpty() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity();

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtDestinationAndSourceIsEmpty() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity();

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNull(stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic2.getMagneticFluxDensityStandardDeviation(),
                0.0, 0.0);
    }

    @Test
    public void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtSourceAndDestination() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std1 = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity1 =
                new BodyMagneticFluxDensity(bx1, by1, bz1);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity1, std1);

        final double bx2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std2 = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity2 =
                new BodyMagneticFluxDensity(bx2, by2, bz2);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity2, std2);

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std1 = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity1 =
                new BodyMagneticFluxDensity(bx1, by1, bz1);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity1, std1);

        final double bx2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std2 = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity2 =
                new BodyMagneticFluxDensity(bx2, by2, bz2);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity2, std2);

        stdMagnetic1.copyFrom(stdMagnetic2);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(),
                stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic3 =
                new StandardDeviationBodyMagneticFluxDensity();

        assertEquals(stdMagnetic1.hashCode(), stdMagnetic2.hashCode());
        assertNotEquals(stdMagnetic1.hashCode(), stdMagnetic3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic3 =
                new StandardDeviationBodyMagneticFluxDensity();

        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertTrue(stdMagnetic1.equals((Object) stdMagnetic1));
        assertTrue(stdMagnetic1.equals(stdMagnetic1));
        assertTrue(stdMagnetic1.equals(stdMagnetic2));
        assertFalse(stdMagnetic1.equals(stdMagnetic3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(stdMagnetic1.equals((Object) null));
        assertFalse(stdMagnetic1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(stdMagnetic1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic2 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);
        final StandardDeviationBodyMagneticFluxDensity stdMagnetic3 =
                new StandardDeviationBodyMagneticFluxDensity();

        assertTrue(stdMagnetic1.equals(stdMagnetic1, THRESHOLD));
        assertTrue(stdMagnetic1.equals(stdMagnetic2, THRESHOLD));
        assertFalse(stdMagnetic1.equals(stdMagnetic3, THRESHOLD));
        assertFalse(stdMagnetic1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY,
                MAX_MAGNETIC_FLUX_DENSITY);
        final double std = randomizer.nextDouble(0.0,
                MAX_MAGNETIC_FLUX_DENSITY);
        final BodyMagneticFluxDensity magneticFluxDensity =
                new BodyMagneticFluxDensity(bx, by, bz);

        final StandardDeviationBodyMagneticFluxDensity stdMagnetic1 =
                new StandardDeviationBodyMagneticFluxDensity(
                        magneticFluxDensity, std);

        final Object stdMagnetic2 = stdMagnetic1.clone();

        // check
        assertEquals(stdMagnetic1, stdMagnetic2);
    }
}
