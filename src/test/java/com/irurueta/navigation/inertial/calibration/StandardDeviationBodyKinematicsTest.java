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

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class StandardDeviationBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default values
        assertNull(stdKinematics.getKinematics());
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        Acceleration f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        AngularSpeed w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);


        // test constructor with body kinematics
        final BodyKinematics kinematics = new BodyKinematics();
        stdKinematics = new StandardDeviationBodyKinematics(kinematics);

        // check default values
        assertSame(stdKinematics.getKinematics(), kinematics);
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);


        // test constructor with specific force and angular rate standard deviations
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        stdKinematics = new StandardDeviationBodyKinematics(specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertNull(stdKinematics.getKinematics());
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), specificForceStandardDeviation, 0.0);
        f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), angularRateStandardDeviation, 0.0);
        w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);

        // Force IllegalArgumentException
        stdKinematics = null;
        try {
            stdKinematics = new StandardDeviationBodyKinematics(-specificForceStandardDeviation,
                    angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            stdKinematics = new StandardDeviationBodyKinematics(specificForceStandardDeviation,
                    -angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdKinematics);


        // test constructor with body kinematics, specific force and
        // angular rate standard deviations
        stdKinematics = new StandardDeviationBodyKinematics(kinematics,
                specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertSame(stdKinematics.getKinematics(), kinematics);
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), specificForceStandardDeviation, 0.0);
        f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), angularRateStandardDeviation, 0.0);
        w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);

        // Force IllegalArgumentException
        stdKinematics = null;
        try {
            stdKinematics = new StandardDeviationBodyKinematics(kinematics,
                    -specificForceStandardDeviation,
                    angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            stdKinematics = new StandardDeviationBodyKinematics(kinematics,
                    specificForceStandardDeviation,
                    -angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdKinematics);


        // test constructor with specific force and angular rate standard deviations
        final Acceleration f = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        stdKinematics = new StandardDeviationBodyKinematics(f, w);

        // check default values
        assertNull(stdKinematics.getKinematics());
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), specificForceStandardDeviation, 0.0);
        f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), angularRateStandardDeviation, 0.0);
        w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);

        // Force IllegalArgumentException
        stdKinematics = null;
        try {
            stdKinematics = new StandardDeviationBodyKinematics(
                    new Acceleration(-specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND),
                    w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            stdKinematics = new StandardDeviationBodyKinematics(f,
                    new AngularSpeed(-angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdKinematics);


        // test constructor with body kinematics, specific force and
        // angular rate standard deviations
        stdKinematics = new StandardDeviationBodyKinematics(kinematics, f, w);

        // check default values
        assertSame(stdKinematics.getKinematics(), kinematics);
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(), specificForceStandardDeviation, 0.0);
        f1 = stdKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(f1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        f2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f2);
        assertEquals(f1, f2);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(), angularRateStandardDeviation, 0.0);
        w1 = stdKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(w1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w2);
        assertEquals(w1, w2);

        // Force IllegalArgumentException
        stdKinematics = null;
        try {
            stdKinematics = new StandardDeviationBodyKinematics(kinematics,
                    new Acceleration(-specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND),
                    w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            stdKinematics = new StandardDeviationBodyKinematics(kinematics, f,
                    new AngularSpeed(-angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(stdKinematics);


        // test copy constructor
        stdKinematics = new StandardDeviationBodyKinematics(kinematics,
                specificForceStandardDeviation,
                angularRateStandardDeviation);

        final StandardDeviationBodyKinematics stdKinematics2 =
                new StandardDeviationBodyKinematics(stdKinematics);

        // check
        assertEquals(stdKinematics.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(),
                stdKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(stdKinematics.getAngularRateStandardDeviation(),
                stdKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testGetSetKinematics() {
        final StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default value
        assertNull(stdKinematics.getKinematics());

        // set new value
        final BodyKinematics kinematics = new BodyKinematics();
        stdKinematics.setKinematics(kinematics);

        // check
        assertSame(stdKinematics.getKinematics(), kinematics);
    }

    @Test
    public void testGetSetSpecificForceStandardDeviation() {
        final StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default value
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        stdKinematics.setSpecificForceStandardDeviation(specificForceStandardDeviation);

        // check
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);

        // Force IllegalArgumentException
        try {
            stdKinematics.setSpecificForceStandardDeviation(-specificForceStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSpecificForceStandardDeviationAsAcceleration() {
        final StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default value
        final Acceleration f1 = stdKinematics
                .getSpecificForceStandardDeviationAsAcceleration();

        assertEquals(f1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(f1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final Acceleration f2 = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        stdKinematics.setSpecificForceStandardDeviation(f2);

        // check
        final Acceleration f3 = stdKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        final Acceleration f4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        stdKinematics.getSpecificForceStandardDeviationAsAcceleration(f4);

        assertEquals(f2, f3);
        assertEquals(f2, f4);

        // Force IllegalArgumentException
        try {
            stdKinematics.setSpecificForceStandardDeviation(
                    new Acceleration(-specificForceStandardDeviation,
                            AccelerationUnit.METERS_PER_SQUARED_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularRateStandardDeviation() {
        final StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default value
        assertEquals(stdKinematics.getSpecificForceStandardDeviation(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        stdKinematics.setAngularRateStandardDeviation(angularRateStandardDeviation);

        // check
        assertEquals(stdKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
    }

    @Test
    public void testGetSetAngularRateStandardDeviationAsAngularSpeed() {
        final StandardDeviationBodyKinematics stdKinematics =
                new StandardDeviationBodyKinematics();

        // check default value
        final AngularSpeed w1 = stdKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();

        assertEquals(w1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(w1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final AngularSpeed w2 = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        stdKinematics.setAngularRateStandardDeviation(w2);

        // check
        final AngularSpeed w3 = stdKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        final AngularSpeed w4 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        stdKinematics.getAngularRateStandardDeviationAsAngularSpeed(w4);

        assertEquals(w2, w3);
        assertEquals(w2, w4);

        // Force IllegalArgumentException
        try {
            stdKinematics.setAngularRateStandardDeviation(
                    new AngularSpeed(-angularRateStandardDeviation,
                            AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCopyFromWhenBodyKinematicsAreAvailableAtSourceAndDestinationIsEmpty() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();

        final StandardDeviationBodyKinematics stdKinematics2 =
                new StandardDeviationBodyKinematics();

        stdKinematics2.copyFrom(stdKinematics1);

        // check
        assertNotSame(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getSpecificForceStandardDeviation(),
                stdKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(stdKinematics1.getAngularRateStandardDeviation(),
                stdKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testCopyFromWhenBodyKinematicsAreAvailableAtDestinationAndSourceIsEmpty() {
        final StandardDeviationBodyKinematics stdKinematics1 =
                new StandardDeviationBodyKinematics();

        final StandardDeviationBodyKinematics stdKinematics2 = createStdKinematics();

        stdKinematics2.copyFrom(stdKinematics1);

        // check
        assertNull(stdKinematics2.getKinematics());
        assertEquals(stdKinematics2.getSpecificForceStandardDeviation(),
                0.0, 0.0);
        assertEquals(stdKinematics2.getAngularRateStandardDeviation(),
                0.0, 0.0);
    }

    @Test
    public void testCopyFromWhenBodyKinematicsAreAvailableAtSourceAndDestination() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();
        final StandardDeviationBodyKinematics stdKinematics2 = createStdKinematics();

        stdKinematics2.copyFrom(stdKinematics1);

        // check
        assertNotSame(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getSpecificForceStandardDeviation(),
                stdKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(stdKinematics1.getAngularRateStandardDeviation(),
                stdKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();
        final StandardDeviationBodyKinematics stdKinematics2 = createStdKinematics();

        stdKinematics1.copyTo(stdKinematics2);

        // check
        assertNotSame(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getKinematics(), stdKinematics2.getKinematics());
        assertEquals(stdKinematics1.getSpecificForceStandardDeviation(),
                stdKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(stdKinematics1.getAngularRateStandardDeviation(),
                stdKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testHashCode() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();
        final StandardDeviationBodyKinematics stdKinematics2 =
                new StandardDeviationBodyKinematics(stdKinematics1);
        final StandardDeviationBodyKinematics stdKinematics3 =
                new StandardDeviationBodyKinematics();

        assertEquals(stdKinematics1.hashCode(), stdKinematics2.hashCode());
        assertNotEquals(stdKinematics1.hashCode(), stdKinematics3.hashCode());
    }

    @Test
    public void testEquals() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();
        final StandardDeviationBodyKinematics stdKinematics2 =
                new StandardDeviationBodyKinematics(stdKinematics1);
        final StandardDeviationBodyKinematics stdKinematics3 =
                new StandardDeviationBodyKinematics();

        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertTrue(stdKinematics1.equals((Object) stdKinematics1));
        //noinspection EqualsWithItself
        assertTrue(stdKinematics1.equals(stdKinematics1));
        assertTrue(stdKinematics1.equals(stdKinematics2));
        assertFalse(stdKinematics1.equals(stdKinematics3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(stdKinematics1.equals((Object) null));
        assertFalse(stdKinematics1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(stdKinematics1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();
        final StandardDeviationBodyKinematics stdKinematics2 =
                new StandardDeviationBodyKinematics(stdKinematics1);
        final StandardDeviationBodyKinematics stdKinematics3 =
                new StandardDeviationBodyKinematics();

        assertTrue(stdKinematics1.equals(stdKinematics1, THRESHOLD));
        assertTrue(stdKinematics1.equals(stdKinematics2, THRESHOLD));
        assertFalse(stdKinematics1.equals(stdKinematics3, THRESHOLD));
        assertFalse(stdKinematics1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final StandardDeviationBodyKinematics stdKinematics1 = createStdKinematics();

        final Object stdKinematics2 = stdKinematics1.clone();

        // check
        assertEquals(stdKinematics1, stdKinematics2);
    }

    private static BodyKinematics createKinematics() {
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

    private static StandardDeviationBodyKinematics createStdKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final BodyKinematics kinematics = createKinematics();
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        return new StandardDeviationBodyKinematics(kinematics,
                specificForceStandardDeviation,
                angularRateStandardDeviation);
    }
}
