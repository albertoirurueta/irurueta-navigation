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
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.assertFalse;

public class BodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        BodyKinematics k = new BodyKinematics();

        // check default values
        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);
        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(k.getSpecificForceNorm(), 0.0, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test constructor with specific force
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        k = new BodyKinematics(fx, fy, fz);

        // check default values
        assertEquals(k.getFx(), fx, 0.0);
        assertEquals(k.getFy(), fy, 0.0);
        assertEquals(k.getFz(), fz, 0.0);
        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), fx, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), fy, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), fz, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        final double normF = Math.sqrt(fx * fx + fy * fy + fz * fz);
        assertEquals(k.getSpecificForceNorm(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test constructor with specific force and angular rate
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        // check default values
        assertEquals(k.getFx(), fx, 0.0);
        assertEquals(k.getFy(), fy, 0.0);
        assertEquals(k.getFz(), fz, 0.0);
        assertEquals(k.getAngularRateX(), angularRateX, 0.0);
        assertEquals(k.getAngularRateY(), angularRateY, 0.0);
        assertEquals(k.getAngularRateZ(), angularRateZ, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), fx, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), fy, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), fz, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), angularRateX, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), angularRateY, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), angularRateZ, 0.0);

        final double angularRateNorm = Math.sqrt(angularRateX * angularRateX +
                angularRateY * angularRateY + angularRateZ * angularRateZ);
        assertEquals(k.getSpecificForceNorm(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test constructor with specific forces accelerations
        final Acceleration specificForceX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k = new BodyKinematics(specificForceX, specificForceY, specificForceZ);

        // check default values
        assertEquals(k.getFx(), fx, 0.0);
        assertEquals(k.getFy(), fy, 0.0);
        assertEquals(k.getFz(), fz, 0.0);
        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), fx, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), fy, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), fz, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(k.getSpecificForceNorm(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test constructor with angular speeds
        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        k = new BodyKinematics(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check default values
        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);
        assertEquals(k.getAngularRateX(), angularRateX, 0.0);
        assertEquals(k.getAngularRateY(), angularRateY, 0.0);
        assertEquals(k.getAngularRateZ(), angularRateZ, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), angularRateX, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), angularRateY, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), angularRateZ, 0.0);

        assertEquals(k.getSpecificForceNorm(), 0.0, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test constructor with specific forces accelerations and angular speeds
        k = new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        // check default values
        assertEquals(k.getFx(), fx, 0.0);
        assertEquals(k.getFy(), fy, 0.0);
        assertEquals(k.getFz(), fz, 0.0);
        assertEquals(k.getAngularRateX(), angularRateX, 0.0);
        assertEquals(k.getAngularRateY(), angularRateY, 0.0);
        assertEquals(k.getAngularRateZ(), angularRateZ, 0.0);

        assertEquals(k.getSpecificForceX().getValue().doubleValue(), fx, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), fy, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), fz, 0.0);
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), angularRateX, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), angularRateY, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), angularRateZ, 0.0);

        assertEquals(k.getSpecificForceNorm(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), normF, 0.0);
        assertEquals(k.getSpecificForceNormAsAcceleration().getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(k.getAngularRateNorm(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getValue().doubleValue(), angularRateNorm, 0.0);
        assertEquals(k.getAngularSpeedNorm().getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);


        // test copy constructor
        final BodyKinematics k2 = new BodyKinematics(k);

        // check default values
        assertEquals(k.getFx(), k2.getFx(), 0.0);
        assertEquals(k.getFy(), k2.getFy(), 0.0);
        assertEquals(k.getFz(), k2.getFz(), 0.0);
        assertEquals(k.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    public void testGetSetFx() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getFx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        k.setFx(fx);

        // check
        assertEquals(k.getFx(), fx, 0.0);
    }

    @Test
    public void testGetSetFy() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getFy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        k.setFy(fy);

        // check
        assertEquals(k.getFy(), fy, 0.0);
    }

    @Test
    public void testGetSetFz() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getFz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        k.setFz(fz);

        // check
        assertEquals(k.getFz(), fz, 0.0);
    }

    @Test
    public void testSetSpecificForceCoordinates() {
        final BodyKinematics k = new BodyKinematics();

        // check default values
        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        k.setSpecificForceCoordinates(fx, fy, fz);

        // check
        assertEquals(k.getFx(), fx, 0.0);
        assertEquals(k.getFy(), fy, 0.0);
        assertEquals(k.getFz(), fz, 0.0);
    }

    @Test
    public void testGetSetSpecificForceX() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getSpecificForceX().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration specificForceX1 = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceX(specificForceX1);

        // check
        final Acceleration specificForceX2 = k.getSpecificForceX();
        final Acceleration specificForceX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceX(specificForceX3);

        assertEquals(specificForceX1, specificForceX2);
        assertEquals(specificForceX1, specificForceX3);
    }

    @Test
    public void testGetSetSpecificForceY() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration specificForceY1 = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceY(specificForceY1);

        // check
        final Acceleration specificForceY2 = k.getSpecificForceY();
        final Acceleration specificForceY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceY(specificForceY3);

        assertEquals(specificForceY1, specificForceY2);
        assertEquals(specificForceY1, specificForceY3);
    }

    @Test
    public void testGetSetSpecificForceZ() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration specificForceZ1 = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceZ(specificForceZ1);

        // check
        final Acceleration specificForceZ2 = k.getSpecificForceZ();
        final Acceleration specificForceZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceZ(specificForceZ3);

        assertEquals(specificForceZ1, specificForceZ2);
        assertEquals(specificForceZ1, specificForceZ3);
    }

    @Test
    public void testSetSpecificForceCoordinatesAsAcceleration() {
        final BodyKinematics k = new BodyKinematics();

        // check default values
        assertEquals(k.getSpecificForceX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getSpecificForceZ().getValue().doubleValue(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final Acceleration specificForceX1 = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceY1 = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceZ1 = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceCoordinates(specificForceX1, specificForceY1, specificForceZ1);

        // check
        final Acceleration specificForceX2 = k.getSpecificForceX();
        final Acceleration specificForceY2 = k.getSpecificForceY();
        final Acceleration specificForceZ2 = k.getSpecificForceZ();
        assertEquals(specificForceX1, specificForceX2);
        assertEquals(specificForceY1, specificForceY2);
        assertEquals(specificForceZ1, specificForceZ2);
    }

    @Test
    public void testGetSetAngularRateX() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularRateX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateX(angularRateX);

        // check
        assertEquals(k.getAngularRateX(), angularRateX, 0.0);
    }

    @Test
    public void testGetSetAngularRateY() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularRateY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateY(angularRateY);

        // check
        assertEquals(k.getAngularRateY(), angularRateY, 0.0);
    }

    @Test
    public void testGetSetAngularRateZ() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateZ(angularRateZ);

        // check
        assertEquals(k.getAngularRateZ(), angularRateZ, 0.0);
    }

    @Test
    public void testSetAngularRateCoordinates() {
        final BodyKinematics k = new BodyKinematics();

        // check default values
        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateCoordinates(angularRateX, angularRateY, angularRateZ);

        // check
        assertEquals(k.getAngularRateX(), angularRateX, 0.0);
        assertEquals(k.getAngularRateY(), angularRateY, 0.0);
        assertEquals(k.getAngularRateZ(), angularRateZ, 0.0);
    }

    @Test
    public void testGetSetAngularSpeedX() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed angularSpeedX1 = new AngularSpeed(angularRateX,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedX(angularSpeedX1);

        // check
        final AngularSpeed angularSpeedX2 = k.getAngularSpeedX();
        final AngularSpeed angularSpeedX3 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedX(angularSpeedX3);

        assertEquals(angularSpeedX1, angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX3);
    }

    @Test
    public void testGetSetAngularSpeedY() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed angularSpeedY1 = new AngularSpeed(angularRateY,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedY(angularSpeedY1);

        // check
        final AngularSpeed angularSpeedY2 = k.getAngularSpeedY();
        final AngularSpeed angularSpeedY3 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedY(angularSpeedY3);

        assertEquals(angularSpeedY1, angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY3);
    }

    @Test
    public void testGetSetAngularSpeedZ() {
        final BodyKinematics k = new BodyKinematics();

        // check default value
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed angularSpeedZ1 = new AngularSpeed(angularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedZ(angularSpeedZ1);

        // check
        final AngularSpeed angularSpeedZ2 = k.getAngularSpeedZ();
        final AngularSpeed angularSpeedZ3 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedZ(angularSpeedZ3);

        assertEquals(angularSpeedZ1, angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ3);
    }

    @Test
    public void testSetAngularSpeedCoordinates() {
        final BodyKinematics k = new BodyKinematics();

        // check default values
        assertEquals(k.getAngularSpeedX().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedY().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(k.getAngularSpeedZ().getValue().doubleValue(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed angularSpeedX1 = new AngularSpeed(angularRateX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY1 = new AngularSpeed(angularRateY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ1 = new AngularSpeed(angularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedCoordinates(angularSpeedX1, angularSpeedY1, angularSpeedZ1);

        // check
        final AngularSpeed angularSpeedX2 = k.getAngularSpeedX();
        final AngularSpeed angularSpeedY2 = k.getAngularSpeedY();
        final AngularSpeed angularSpeedZ2 = k.getAngularSpeedZ();
        assertEquals(angularSpeedX1, angularSpeedX2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
    }

    @Test
    public void testSpecificForceNorm() {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final double normF = Math.sqrt(fx * fx + fy * fy + fz * fz);
        assertEquals(k.getSpecificForceNorm(), normF, 0.0);

        final Acceleration norm1 = k.getSpecificForceNormAsAcceleration();
        final Acceleration norm2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceNormAsAcceleration(norm2);

        assertEquals(norm1.getValue().doubleValue(), normF, 0.0);
        assertEquals(norm1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(norm1, norm2);
    }

    @Test
    public void testAngularRateNorm() {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final double normAngularRate = Math.sqrt(angularRateX * angularRateX +
                angularRateY * angularRateY + angularRateZ * angularRateZ);
        assertEquals(k.getAngularRateNorm(), normAngularRate, 0.0);

        final AngularSpeed norm1 = k.getAngularSpeedNorm();
        final AngularSpeed norm2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedNorm(norm2);

        assertEquals(norm1.getValue().doubleValue(), normAngularRate, 0.0);
        assertEquals(norm1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(norm1, norm2);
    }

    @Test
    public void testCopyTo() {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final BodyKinematics k2 = new BodyKinematics();
        k1.copyTo(k2);

        // check
        assertEquals(k1.getFx(), k2.getFx(), 0.0);
        assertEquals(k1.getFy(), k2.getFy(), 0.0);
        assertEquals(k1.getFz(), k2.getFz(), 0.0);
        assertEquals(k1.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k1.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k1.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    public void testCopyFrom() {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final BodyKinematics k2 = new BodyKinematics();
        k2.copyFrom(k1);

        // check
        assertEquals(k1.getFx(), k2.getFx(), 0.0);
        assertEquals(k1.getFy(), k2.getFy(), 0.0);
        assertEquals(k1.getFz(), k2.getFz(), 0.0);
        assertEquals(k1.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k1.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k1.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    public void testAsSpecificForceArray() {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final double[] array1 = new double[BodyKinematics.COMPONENTS];
        k.asSpecificForceArray(array1);
        final double[] array2 = k.asSpecificForceArray();

        // check
        assertEquals(array1[0], fx, 0.0);
        assertEquals(array1[1], fy, 0.0);
        assertEquals(array1[2], fz, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            k.asSpecificForceArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testAsSpecificForceMatrix() throws WrongSizeException {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Matrix m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        k.asSpecificForceMatrix(m1);
        final Matrix m2 = k.asSpecificForceMatrix();
        final Matrix m3 = new Matrix(1, 1);
        k.asSpecificForceMatrix(m3);

        // check
        assertEquals(m1.getElementAtIndex(0), fx, 0.0);
        assertEquals(m1.getElementAtIndex(1), fy, 0.0);
        assertEquals(m1.getElementAtIndex(2), fz, 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    public void testAsAngularRateArray() {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final double[] array1 = new double[BodyKinematics.COMPONENTS];
        k.asAngularRateArray(array1);
        final double[] array2 = k.asAngularRateArray();

        // check
        assertEquals(array1[0], angularRateX, 0.0);
        assertEquals(array1[1], angularRateY, 0.0);
        assertEquals(array1[2], angularRateZ, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            k.asAngularRateArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testAsAngularRateMatrix() throws WrongSizeException {
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

        final BodyKinematics k = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Matrix m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        k.asAngularRateMatrix(m1);
        final Matrix m2 = k.asAngularRateMatrix();
        final Matrix m3 = new Matrix(1, 1);
        k.asAngularRateMatrix(m3);

        // check
        assertEquals(m1.getElementAtIndex(0), angularRateX, 0.0);
        assertEquals(m1.getElementAtIndex(1), angularRateY, 0.0);
        assertEquals(m1.getElementAtIndex(2), angularRateZ, 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    public void testHashCode() {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k2 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k3 = new BodyKinematics();

        assertEquals(k1.hashCode(), k2.hashCode());
        assertNotEquals(k1.hashCode(), k3.hashCode());
    }

    @Test
    public void testEquals() {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k2 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k3 = new BodyKinematics();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(k1.equals((Object)k1));
        assertTrue(k1.equals(k1));
        assertTrue(k1.equals(k2));
        assertFalse(k1.equals(k3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(k1.equals((Object)null));
        assertFalse(k1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(k1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k2 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final BodyKinematics k3 = new BodyKinematics();

        assertTrue(k1.equals(k1, THRESHOLD));
        assertTrue(k1.equals(k2, THRESHOLD));
        assertFalse(k1.equals(k3, THRESHOLD));
        assertFalse(k1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
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

        final BodyKinematics k1 = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Object k2 = k1.clone();

        // check
        assertEquals(k1, k2);
    }
}
