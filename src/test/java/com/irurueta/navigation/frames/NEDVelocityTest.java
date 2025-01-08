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
package com.irurueta.navigation.frames;

import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class NEDVelocityTest {

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {

        // test empty constructor
        var velocity = new NEDVelocity();

        // check
        assertEquals(0.0, velocity.getVn(), 0.0);
        assertEquals(0.0, velocity.getVe(), 0.0);
        assertEquals(0.0, velocity.getVd(), 0.0);

        assertEquals(0.0, velocity.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, velocity.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, velocity.getSpeedD().getValue().doubleValue(), 0.0);

        // test constructor with coordinates
        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        velocity = new NEDVelocity(vn, ve, vd);

        // check
        assertEquals(vn, velocity.getVn(), 0.0);
        assertEquals(ve, velocity.getVe(), 0.0);
        assertEquals(vd, velocity.getVd(), 0.0);

        assertEquals(vn, velocity.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, velocity.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, velocity.getSpeedD().getValue().doubleValue(), 0.0);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        velocity = new NEDVelocity(speedN, speedE, speedD);

        // check
        assertEquals(vn, velocity.getVn(), 0.0);
        assertEquals(ve, velocity.getVe(), 0.0);
        assertEquals(vd, velocity.getVd(), 0.0);

        assertEquals(vn, velocity.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, velocity.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, velocity.getSpeedD().getValue().doubleValue(), 0.0);

        // test copy constructor
        final var velocity2 = new NEDVelocity(velocity);

        // check
        assertEquals(velocity.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    void testGetSetVn() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial value
        assertEquals(0.0, velocity.getVn(), 0.0);

        // set new value
        velocity.setVn(vn);

        // check
        assertEquals(vn, velocity.getVn(), 0.0);
    }

    @Test
    void testGetSetVe() {

        final var randomizer = new UniformRandomizer();
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial value
        assertEquals(0.0, velocity.getVe(), 0.0);

        // set new value
        velocity.setVe(ve);

        // check
        assertEquals(ve, velocity.getVe(), 0.0);
    }

    @Test
    void testGetSetVd() {

        final var randomizer = new UniformRandomizer();
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial value
        assertEquals(0.0, velocity.getVd(), 0.0);

        // set new value
        velocity.setVd(vd);

        // check
        assertEquals(vd, velocity.getVd(), 0.0);
    }

    @Test
    void testSetCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial values
        assertEquals(0.0, velocity.getVn(), 0.0);
        assertEquals(0.0, velocity.getVe(), 0.0);
        assertEquals(0.0, velocity.getVd(), 0.0);

        // set new values
        velocity.setCoordinates(vn, ve, vd);

        // check
        assertEquals(vn, velocity.getVn(), 0.0);
        assertEquals(ve, velocity.getVe(), 0.0);
        assertEquals(vd, velocity.getVd(), 0.0);
    }

    @Test
    void testGetNorm() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var norm = Math.sqrt(vn * vn + ve * ve + vd * vd);

        final var velocity = new NEDVelocity(vn, ve, vd);

        assertEquals(norm, velocity.getNorm(), 0.0);
    }

    @Test
    void testGetNormAsSpeed() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var norm = Math.sqrt(vn * vn + ve * ve + vd * vd);

        final var velocity = new NEDVelocity(vn, ve, vd);

        final var norm1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getNormAsSpeed(norm1);
        final var norm2 = velocity.getNormAsSpeed();

        assertEquals(norm, norm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, norm1.getUnit());

        assertEquals(norm1, norm2);
    }

    @Test
    void testGetSetSpeedN() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial values
        assertEquals(0.0, velocity.getVn(), 0.0);
        assertEquals(0.0, velocity.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocity.getSpeedN().getUnit());

        // set new value
        final var speedN1 = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedN(speedN1);

        // check
        final var speedN2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedN(speedN2);
        final var speedN3 = velocity.getSpeedN();

        assertEquals(vn, speedN2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedN2.getUnit());
        assertEquals(speedN2, speedN3);
    }

    @Test
    void testGetSetSpeedE() {

        final var randomizer = new UniformRandomizer();
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial values
        assertEquals(0.0, velocity.getVe(), 0.0);
        assertEquals(0.0, velocity.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocity.getSpeedE().getUnit());

        // set new value
        final var speedE1 = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedE(speedE1);

        // check
        final var speedE2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedE(speedE2);
        final var speedE3 = velocity.getSpeedE();

        assertEquals(ve, speedE2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedE2.getUnit());
        assertEquals(speedE2, speedE3);
    }

    @Test
    void testGetSetSpeedD() {

        final var randomizer = new UniformRandomizer();
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity();

        // check initial values
        assertEquals(0.0, velocity.getVd(), 0.0);
        assertEquals(0.0, velocity.getSpeedD().getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocity.getSpeedD().getUnit());

        // set new value
        final var speedD1 = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        velocity.setSpeedD(speedD1);

        // check
        final var speedD2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        velocity.getSpeedD(speedD2);
        final var speedD3 = velocity.getSpeedD();

        assertEquals(vd, speedD2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedD2.getUnit());
        assertEquals(speedD2, speedD3);
    }

    @Test
    void testSetSpeedCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final var velocity = new NEDVelocity();

        // check initial values
        assertEquals(0.0, velocity.getVn(), 0.0);
        assertEquals(0.0, velocity.getVe(), 0.0);
        assertEquals(0.0, velocity.getVd(), 0.0);

        // set new values
        velocity.setCoordinates(speedN, speedE, speedD);

        // check
        assertEquals(vn, velocity.getVn(), 0.0);
        assertEquals(ve, velocity.getVe(), 0.0);
        assertEquals(vd, velocity.getVd(), 0.0);
    }

    @Test
    void testCopyTo() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        final var velocity2 = new NEDVelocity();
        velocity1.copyTo(velocity2);

        // check
        assertEquals(velocity1.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity1.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity1.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    void testCopyFrom() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        final var velocity2 = new NEDVelocity();
        velocity2.copyFrom(velocity1);

        // check
        assertEquals(velocity1.getVn(), velocity2.getVn(), 0.0);
        assertEquals(velocity1.getVe(), velocity2.getVe(), 0.0);
        assertEquals(velocity1.getVd(), velocity2.getVd(), 0.0);
    }

    @Test
    void testHashCode() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        final var velocity2 = new NEDVelocity(vn, ve, vd);
        final var velocity3 = new NEDVelocity();

        assertEquals(velocity1.hashCode(), velocity2.hashCode());
        assertNotEquals(velocity1.hashCode(), velocity3.hashCode());
    }

    @Test
    void testEquals() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        final var velocity2 = new NEDVelocity(vn, ve, vd);
        final var velocity3 = new NEDVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(velocity1.equals((Object) velocity1));
        //noinspection EqualsWithItself
        assertTrue(velocity1.equals(velocity1));
        assertTrue(velocity1.equals(velocity2));
        assertFalse(velocity1.equals(velocity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(velocity1.equals((Object) null));
        assertFalse(velocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), velocity1);
    }

    @Test
    void testEqualsWithThreshold() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        final var velocity2 = new NEDVelocity(vn, ve, vd);
        final var velocity3 = new NEDVelocity();

        assertTrue(velocity1.equals(velocity1, THRESHOLD));
        assertTrue(velocity1.equals(velocity2, THRESHOLD));
        assertFalse(velocity1.equals(velocity3, THRESHOLD));
        assertFalse(velocity1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);

        final var velocity2 = velocity1.clone();

        assertEquals(velocity1, velocity2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity1 = new NEDVelocity(vn, ve, vd);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(velocity1);
        final var velocity2 = SerializationHelper.<NEDVelocity>deserialize(bytes);

        // check
        assertEquals(velocity1, velocity2);
        assertNotSame(velocity1, velocity2);
    }
}
