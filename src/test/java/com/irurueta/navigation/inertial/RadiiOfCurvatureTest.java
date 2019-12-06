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

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class RadiiOfCurvatureTest {

    private static final double MIN_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;
    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default values
        assertEquals(radii.getRn(), 0.0, 0.0);
        assertEquals(radii.getRe(), 0.0, 0.0);

        assertEquals(radii.getRnDistance().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(radii.getReDistance().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        radii = new RadiiOfCurvature(rn, re);

        // check default values
        assertEquals(radii.getRn(), rn, 0.0);
        assertEquals(radii.getRe(), re, 0.0);

        assertEquals(radii.getRnDistance().getValue().doubleValue(), rn, 0.0);
        assertEquals(radii.getReDistance().getValue().doubleValue(), re, 0.0);


        // test constructor with distance values
        final Distance rnDistance = new Distance(rn, DistanceUnit.METER);
        final Distance reDistance = new Distance(re, DistanceUnit.METER);

        radii = new RadiiOfCurvature(rnDistance, reDistance);

        // check default values
        assertEquals(radii.getRn(), rn, 0.0);
        assertEquals(radii.getRe(), re, 0.0);

        assertEquals(radii.getRnDistance().getValue().doubleValue(), rn, 0.0);
        assertEquals(radii.getReDistance().getValue().doubleValue(), re, 0.0);

        // test constructor from another instance
        final RadiiOfCurvature radii2 = new RadiiOfCurvature(radii);

        // check default values
        assertEquals(radii2.getRn(), rn, 0.0);
        assertEquals(radii2.getRe(), re, 0.0);
    }

    @Test
    public void testGetSetRn() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default value
        assertEquals(radii.getRn(), 0.0, 0.0);

        // set new value
        radii.setRn(rn);

        // check
        assertEquals(radii.getRn(), rn, 0.0);
    }

    @Test
    public void testGetSetRe() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default value
        assertEquals(radii.getRe(), 0.0, 0.0);

        // set new value
        radii.setRe(re);

        // check
        assertEquals(radii.getRe(), re, 0.0);
    }

    @Test
    public void testSetRadii1() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default values
        assertEquals(radii.getRn(), 0.0, 0.0);
        assertEquals(radii.getRe(), 0.0, 0.0);

        // set values
        radii.setValues(rn, re);

        // check
        assertEquals(radii.getRn(), rn, 0.0);
        assertEquals(radii.getRe(), re, 0.0);
    }

    @Test
    public void testGetSetRnDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default value
        assertEquals(radii.getRnDistance().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance rnDistance1 = new Distance(rn, DistanceUnit.METER);
        radii.setRnDistance(rnDistance1);

        // check
        final Distance rnDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        radii.getRnDistance(rnDistance2);
        final Distance rnDistance3 = radii.getRnDistance();

        assertEquals(rnDistance1, rnDistance2);
        assertEquals(rnDistance1, rnDistance3);
    }

    @Test
    public void testGetSetReDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default value
        assertEquals(radii.getReDistance().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance reDistance1 = new Distance(re, DistanceUnit.METER);
        radii.setReDistance(reDistance1);

        // check
        final Distance reDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        radii.getReDistance(reDistance2);
        final Distance reDistance3 = radii.getReDistance();

        assertEquals(reDistance1, reDistance2);
        assertEquals(reDistance1, reDistance3);
    }

    @Test
    public void testSetRadii2() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii = new RadiiOfCurvature();

        // check default values
        assertEquals(radii.getRnDistance().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(radii.getReDistance().getValue().doubleValue(), 0.0, 0.0);

        // set values
        final Distance rnDistance = new Distance(rn, DistanceUnit.METER);
        final Distance reDistance = new Distance(re, DistanceUnit.METER);
        radii.setValues(rnDistance, reDistance);

        // check
        assertEquals(radii.getRn(), rn, 0.0);
        assertEquals(radii.getRe(), re, 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii2 = new RadiiOfCurvature();

        radii1.copyTo(radii2);

        // check
        assertEquals(radii1.getRn(), radii2.getRn(), 0.0);
        assertEquals(radii1.getRe(), radii2.getRe(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii2 = new RadiiOfCurvature();

        radii2.copyFrom(radii1);

        // check
        assertEquals(radii1.getRn(), radii2.getRn(), 0.0);
        assertEquals(radii1.getRe(), radii2.getRe(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii2 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii3 = new RadiiOfCurvature();

        assertEquals(radii1.hashCode(), radii2.hashCode());
        assertNotEquals(radii1.hashCode(), radii3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii2 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii3 = new RadiiOfCurvature();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(radii1.equals((Object)radii1));
        assertTrue(radii1.equals(radii1));
        assertTrue(radii1.equals(radii2));
        assertFalse(radii1.equals(radii3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(radii1.equals((Object)null));
        assertFalse(radii1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(radii1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii2 = new RadiiOfCurvature(rn, re);
        final RadiiOfCurvature radii3 = new RadiiOfCurvature();

        assertTrue(radii1.equals(radii1, THRESHOLD));
        assertTrue(radii1.equals(radii2, THRESHOLD));
        assertFalse(radii1.equals(radii3, THRESHOLD));
        assertFalse(radii1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final RadiiOfCurvature radii1 = new RadiiOfCurvature(rn, re);

        final Object radii2 = radii1.clone();

        // check
        assertEquals(radii1, radii2);
    }
}
