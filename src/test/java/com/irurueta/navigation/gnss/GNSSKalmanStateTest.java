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
package com.irurueta.navigation.gnss;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GNSSKalmanStateTest {

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double THRESHOLD = 1e-8;

    @Test
    public void testConstructor() throws WrongSizeException {
        // test empty constructor
        GNSSKalmanState state = new GNSSKalmanState();

        // check default values
        assertNull(state.getEstimation());
        assertNull(state.getCovariance());


        // test constructor with estimation and covariance
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        state = new GNSSKalmanState(estimation, covariance);

        // check default values
        assertSame(state.getEstimation(), estimation);
        assertSame(state.getCovariance(), covariance);

        // Force IllegalArgumentException
        state = null;
        try {
            state = new GNSSKalmanState(estimation, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(state);


        // test copy constructor
        state = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state2 = new GNSSKalmanState(state);

        // check default values
        assertEquals(state2.getEstimation(), estimation);
        assertEquals(state2.getCovariance(), covariance);
    }

    @Test
    public void testGetEstimation() {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertNull(state.getEstimation());
        assertFalse(state.getEstimation(new GNSSEstimation()));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        state.setEstimation(estimation1);

        // check
        final GNSSEstimation estimation2 = new GNSSEstimation();
        assertTrue(state.getEstimation(estimation2));
        final GNSSEstimation estimation3 = state.getEstimation();

        assertEquals(estimation1, estimation2);
        assertEquals(estimation1, estimation3);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final GNSSKalmanState state = new GNSSKalmanState();

        // check default value
        assertNull(state.getCovariance());
        assertFalse(state.getCovariance(new Matrix(1, 1)));

        // set new value
        final Matrix covariance1 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        state.setCovariance(covariance1);

        // check
        final Matrix covariance2 = new Matrix(1, 1);
        assertTrue(state.getCovariance(covariance2));
        final Matrix covariance3 = state.getCovariance();

        assertEquals(covariance1, covariance2);
        assertEquals(covariance1, covariance3);
    }

    @Test
    public void testCopyToWhenInputHasValuesAndOutputDoesNotHaveValues()
            throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation, covariance);

        assertEquals(state1.getEstimation(), estimation);
        assertEquals(state1.getCovariance(), covariance);

        final GNSSKalmanState state2 = new GNSSKalmanState();
        state1.copyTo(state2);


        assertEquals(state2.getEstimation(), estimation);
        assertEquals(state2.getCovariance(), covariance);
    }

    @Test
    public void testCopyToWhenInputHasNoValuesAndOutputHasValues()
            throws WrongSizeException {
        final GNSSKalmanState state1 = new GNSSKalmanState();

        assertNull(state1.getEstimation());
        assertNull(state1.getCovariance());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);

        final GNSSKalmanState state2 = new GNSSKalmanState(estimation, covariance);
        state1.copyTo(state2);

        assertNull(state2.getEstimation());
        assertNull(state2.getCovariance());
    }

    @Test
    public void testCopyToWhenInputHasNoValuesAndOutputDoesNotHaveValues() {
        final GNSSKalmanState state1 = new GNSSKalmanState();

        assertNull(state1.getEstimation());
        assertNull(state1.getCovariance());

        final GNSSKalmanState state2 = new GNSSKalmanState();
        state1.copyTo(state2);

        assertNull(state2.getEstimation());
        assertNull(state2.getCovariance());
    }

    @Test
    public void testCopyToWhenBothInputAndOutputHaveValues() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation1 = new GNSSEstimation(x1, y1, z1, vx1, vy1, vz1,
                clockOffset1, clockDrift1);
        final Matrix covariance1 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation1, covariance1);


        final double x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation2 = new GNSSEstimation(x2, y2, z2, vx2, vy2, vz2,
                clockOffset2, clockDrift2);
        final Matrix covariance2 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state2 = new GNSSKalmanState(estimation2, covariance2);

        state1.copyTo(state2);

        assertEquals(state2.getEstimation(), estimation1);
        assertEquals(state2.getCovariance(), covariance1);
    }

    @Test
    public void testCopyFrom() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation1 = new GNSSEstimation(x1, y1, z1, vx1, vy1, vz1,
                clockOffset1, clockDrift1);
        final Matrix covariance1 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation1, covariance1);


        final double x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation2 = new GNSSEstimation(x2, y2, z2, vx2, vy2, vz2,
                clockOffset2, clockDrift2);
        final Matrix covariance2 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state2 = new GNSSKalmanState(estimation2, covariance2);

        state2.copyFrom(state1);

        assertEquals(state2.getEstimation(), estimation1);
        assertEquals(state2.getCovariance(), covariance1);
    }

    @Test
    public void testHashCode() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state2 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        assertEquals(state1.hashCode(), state2.hashCode());
        assertNotEquals(state1.hashCode(), state3.hashCode());
    }

    @Test
    public void testEquals() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state2 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(state1.equals((Object) state1));
        assertTrue(state1.equals(state1));
        assertTrue(state1.equals(state2));
        assertFalse(state1.equals(state3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(state1.equals((Object) null));
        assertFalse(state1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(state1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state2 = new GNSSKalmanState(estimation, covariance);
        final GNSSKalmanState state3 = new GNSSKalmanState();

        assertTrue(state1.equals(state1, THRESHOLD));
        assertTrue(state1.equals(state2, THRESHOLD));
        assertFalse(state1.equals(state3, THRESHOLD));
        assertFalse(state1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws WrongSizeException, CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz,
                clockOffset, clockDrift);
        final Matrix covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS,
                GNSSEstimation.NUM_PARAMETERS);
        final GNSSKalmanState state1 = new GNSSKalmanState(estimation, covariance);

        final Object state2 = state1.clone();

        assertEquals(state1, state2);
    }
}
