package com.irurueta.navigation.inertial;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class KinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        Kinematics k = new Kinematics();

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


        // test constructor with specific force
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        k = new Kinematics(fx, fy, fz);

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


        // test constructor with specific force and angular rate
        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        k = new Kinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

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


        // test constructor with specific forces accelerations
        final Acceleration specificForceX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration specificForceZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k = new Kinematics(specificForceX, specificForceY, specificForceZ);

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


        // test constructor with angular speeds
        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        k = new Kinematics(angularSpeedX, angularSpeedY, angularSpeedZ);

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


        // test constructor with specific forces accelerations and angular speeds
        k = new Kinematics(specificForceX, specificForceY, specificForceZ,
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


        // test copy constructor
        Kinematics k2 = new Kinematics(k);

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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
        final Kinematics k = new Kinematics();

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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Kinematics k2 = new Kinematics();
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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Kinematics k2 = new Kinematics();
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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k2 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k3 = new Kinematics();

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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k2 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k3 = new Kinematics();

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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k2 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Kinematics k3 = new Kinematics();

        assertTrue(k1.equals(k1, THRESHOLD));
        assertTrue(k1.equals(k2, THRESHOLD));
        assertFalse(k1.equals(k3, THRESHOLD));
        assertFalse(k1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
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

        final Kinematics k1 = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Object k2 = k1.clone();

        // check
        assertEquals(k1, k2);
    }
}
