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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class BodyKinematicsAndMagneticFluxDensityTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.
    private static final double MIN_MAGNETIC_FLUX_VALUE = 30e-6;
    private static final double MAX_MAGNETIC_FLUX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor1() {
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb.getKinematics());
        assertNull(kb.getMagneticFluxDensity());
    }

    @Test
    public void testConstructor2() {
        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity(kinematics);

        // check default values
        assertSame(kb.getKinematics(), kinematics);
        assertNull(kb.getMagneticFluxDensity());
    }

    @Test
    public void testConstructor3() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity(b);

        // check default values
        assertNull(kb.getKinematics());
        assertSame(kb.getMagneticFluxDensity(), b);
    }

    @Test
    public void testConstructor4() {
        final BodyKinematics kinematics = new BodyKinematics();
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        // check default values
        assertSame(kb.getKinematics(), kinematics);
        assertSame(kb.getMagneticFluxDensity(), b);
    }

    @Test
    public void testConstructor5() {
        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kb1);

        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    public void testGetSetKinematics() {
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(kb.getKinematics());

        // set new value
        final BodyKinematics kinematics = new BodyKinematics();
        kb.setKinematics(kinematics);

        // check
        assertSame(kb.getKinematics(), kinematics);
    }

    @Test
    public void testGetSetMagneticFluxDensity() {
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(kb.getMagneticFluxDensity());

        // set new value
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        kb.setMagneticFluxDensity(b);

        // check
        assertSame(kb.getMagneticFluxDensity(), b);
    }

    @Test
    public void testCopyFromWhenHasBothKinematicsAndMagneticFlux() {
        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    public void testCopyFromWhenHasKinematics() {
        final BodyKinematics kinematics = createKinematics();
        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1, kb2);
    }

    @Test
    public void testCopyFromWhenHasMagneticFlux() {
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(b);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertNull(kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    public void testCopyFromWhenNullData() {
        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity();

        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        // check default values
        assertSame(kb2.getKinematics(), kinematics);
        assertSame(kb2.getMagneticFluxDensity(), b);

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());
    }

    @Test
    public void testCopyTo() {
        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        kb1.copyTo(kb2);

        // check
        assertEquals(kb1.getKinematics(), kinematics1);
        assertEquals(kb1.getMagneticFluxDensity(), b1);
    }

    @Test
    public void testHashCode() {
        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb3 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        assertEquals(kb1.hashCode(), kb2.hashCode());
        assertNotEquals(kb1.hashCode(), kb3.hashCode());
    }

    @Test
    public void testEquals() {
        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb3 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        //noinspection ConstantConditions,SimplifiableAssertion
        assertTrue(kb1.equals((Object) kb1));
        assertTrue(kb1.equals(kb1));
        assertTrue(kb1.equals(kb2));
        assertFalse(kb1.equals(kb3));
        //noinspection ConstantConditions,SimplifiableAssertion,SimplifiableAssertion
        assertFalse(kb1.equals((Object) null));
        assertFalse(kb1.equals(null));
        //noinspection SimplifiableAssertion
        assertFalse(kb1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb2 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final BodyKinematicsAndMagneticFluxDensity kb3 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        assertTrue(kb1.equals(kb1, THRESHOLD));
        assertTrue(kb1.equals(kb2, THRESHOLD));
        assertFalse(kb1.equals(kb3, THRESHOLD));
        assertFalse(kb1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();

        final BodyKinematicsAndMagneticFluxDensity kb1 =
                new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final Object kb2 = kb1.clone();

        // check
        assertEquals(kb1, kb2);
    }


    private BodyKinematics createKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    private BodyMagneticFluxDensity createMagneticFlux() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double bx = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final double by = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final double bz = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);

        return new BodyMagneticFluxDensity(bx, by,bz);
    }
}
