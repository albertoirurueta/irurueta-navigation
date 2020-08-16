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

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class NEDVelocityTest {

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {

        // test empty constructor
        NEDVelocity velocity = new NEDVelocity();

        // check
        assertEquals(velocity.getVn(), 0.0, 0.0);
        assertEquals(velocity.getVe(), 0.0, 0.0);
        assertEquals(velocity.getVd(), 0.0, 0.0);

        assertEquals(velocity.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocity.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocity.getSpeedD().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        velocity = new NEDVelocity(vn, ve, vd);

        // check
        assertEquals(velocity.getVn(), vn, 0.0);
        assertEquals(velocity.getVe(), ve, 0.0);
        assertEquals(velocity.getVd(), vd, 0.0);

        assertEquals(velocity.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(velocity.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(velocity.getSpeedD().getValue().doubleValue(), vd, 0.0);


        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        velocity = new NEDVelocity(speedN, speedE, speedD);

        // check
        assertEquals(velocity.getVn(), vn, 0.0);
        assertEquals(velocity.getVe(), ve, 0.0);
        assertEquals(velocity.getVd(), vd, 0.0);

        assertEquals(velocity.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(velocity.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(velocity.getSpeedD().getValue().doubleValue(), vd, 0.0);


        // test copy constructor
        final NEDVelocity velocity2 = new NEDVelocity(velocity);

        // check
        assertEquals(velocity.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    public void testGetSetVn() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial value
        assertEquals(velocity.getVn(), 0.0, 0.0);

        // set new value
        velocity.setVn(vn);

        // check
        assertEquals(velocity.getVn(), vn, 0.0);
    }

    @Test
    public void testGetSetVe() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial value
        assertEquals(velocity.getVe(), 0.0, 0.0);

        // set new value
        velocity.setVe(ve);

        // check
        assertEquals(velocity.getVe(), ve, 0.0);
    }

    @Test
    public void testGetSetVd() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial value
        assertEquals(velocity.getVd(), 0.0, 0.0);

        // set new value
        velocity.setVd(vd);

        // check
        assertEquals(velocity.getVd(), vd, 0.0);
    }

    @Test
    public void testSetCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial values
        assertEquals(velocity.getVn(), 0.0, 0.0);
        assertEquals(velocity.getVe(), 0.0, 0.0);
        assertEquals(velocity.getVd(), 0.0, 0.0);

        // set new values
        velocity.setCoordinates(vn, ve, vd);

        // check
        assertEquals(velocity.getVn(), vn, 0.0);
        assertEquals(velocity.getVe(), ve, 0.0);
        assertEquals(velocity.getVd(), vd, 0.0);
    }

    @Test
    public void testGetNorm() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double norm = Math.sqrt(vn * vn + ve * ve + vd * vd);

        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);

        assertEquals(velocity.getNorm(), norm, 0.0);
    }

    @Test
    public void testGetNormAsSpeed() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double norm = Math.sqrt(vn * vn + ve * ve + vd * vd);

        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);

        final Speed norm1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getNormAsSpeed(norm1);
        final Speed norm2 = velocity.getNormAsSpeed();

        assertEquals(norm1.getValue().doubleValue(), norm, 0.0);
        assertEquals(norm1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        assertEquals(norm1, norm2);
    }

    @Test
    public void testGetSetSpeedN() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial values
        assertEquals(velocity.getVn(), 0.0, 0.0);
        assertEquals(velocity.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocity.getSpeedN().getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedN1 = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedN(speedN1);

        // check
        final Speed speedN2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedN(speedN2);
        final Speed speedN3 = velocity.getSpeedN();

        assertEquals(speedN2.getValue().doubleValue(), vn, 0.0);
        assertEquals(speedN2.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedN2, speedN3);
    }

    @Test
    public void testGetSetSpeedE() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial values
        assertEquals(velocity.getVe(), 0.0, 0.0);
        assertEquals(velocity.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocity.getSpeedE().getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedE1 = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedE(speedE1);

        // check
        final Speed speedE2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedE(speedE2);
        final Speed speedE3 = velocity.getSpeedE();

        assertEquals(speedE2.getValue().doubleValue(), ve, 0.0);
        assertEquals(speedE2.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedE2, speedE3);
    }

    @Test
    public void testGetSetSpeedD() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial values
        assertEquals(velocity.getVd(), 0.0, 0.0);
        assertEquals(velocity.getSpeedD().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocity.getSpeedD().getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final Speed speedD1 = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedD(speedD1);

        // check
        final Speed speedD2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedD(speedD2);
        final Speed speedD3 = velocity.getSpeedD();

        assertEquals(speedD2.getValue().doubleValue(), vd, 0.0);
        assertEquals(speedD2.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedD2, speedD3);
    }

    @Test
    public void testSetSpeedCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final NEDVelocity velocity = new NEDVelocity();

        // check initial values
        assertEquals(velocity.getVn(), 0.0, 0.0);
        assertEquals(velocity.getVe(), 0.0, 0.0);
        assertEquals(velocity.getVd(), 0.0, 0.0);

        // set new values
        velocity.setCoordinates(speedN, speedE, speedD);

        // check
        assertEquals(velocity.getVn(), vn, 0.0);
        assertEquals(velocity.getVe(), ve, 0.0);
        assertEquals(velocity.getVd(), vd, 0.0);
    }

    @Test
    public void testCopyTo() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity2 = new NEDVelocity();
        velocity1.copyTo(velocity2);

        // check
        assertEquals(velocity1.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity1.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity1.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    public void testCopyFrom() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity2 = new NEDVelocity();
        velocity2.copyFrom(velocity1);

        // check
        assertEquals(velocity1.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity1.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity1.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    public void testHashCode() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity2 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity3 = new NEDVelocity();

        assertEquals(velocity1.hashCode(), velocity2.hashCode());
        assertNotEquals(velocity1.hashCode(), velocity3.hashCode());
    }

    @Test
    public void testEquals() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity2 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity3 = new NEDVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(velocity1.equals((Object) velocity1));
        assertTrue(velocity1.equals(velocity1));
        assertTrue(velocity1.equals(velocity2));
        assertFalse(velocity1.equals(velocity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(velocity1.equals((Object) null));
        assertFalse(velocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(velocity1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity2 = new NEDVelocity(vn, ve, vd);
        final NEDVelocity velocity3 = new NEDVelocity();

        assertTrue(velocity1.equals(velocity1, THRESHOLD));
        assertTrue(velocity1.equals(velocity2, THRESHOLD));
        assertFalse(velocity1.equals(velocity3, THRESHOLD));
        assertFalse(velocity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);

        final Object velocity2 = velocity1.clone();

        assertEquals(velocity1, velocity2);
    }
}
