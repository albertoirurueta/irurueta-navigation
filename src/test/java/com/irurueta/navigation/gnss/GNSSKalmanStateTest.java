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
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class GNSSKalmanStateTest {

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double THRESHOLD = 1e-8;

    @Test
    void testConstructor() throws WrongSizeException {
        // test empty constructor
        var state = new GNSSKalmanState();

        // check default values
        assertNull(state.getEstimation());
        assertNull(state.getCovariance());

        // test constructor with estimation and covariance
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        state = new GNSSKalmanState(estimation, covariance);

        // check default values
        assertSame(state.getEstimation(), estimation);
        assertSame(state.getCovariance(), covariance);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new GNSSKalmanState(estimation, m));

        // test copy constructor
        state = new GNSSKalmanState(estimation, covariance);
        final var state2 = new GNSSKalmanState(state);

        // check default values
        assertEquals(state2.getEstimation(), estimation);
        assertEquals(state2.getCovariance(), covariance);
    }

    @Test
    void testGetEstimation() {
        final var state = new GNSSKalmanState();

        // check default value
        assertNull(state.getEstimation());
        assertFalse(state.getEstimation(new GNSSEstimation()));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        state.setEstimation(estimation1);

        // check
        final var estimation2 = new GNSSEstimation();
        assertTrue(state.getEstimation(estimation2));
        final var estimation3 = state.getEstimation();

        assertEquals(estimation1, estimation2);
        assertEquals(estimation1, estimation3);
    }

    @Test
    void testGetSetCovariance() throws WrongSizeException {
        final var state = new GNSSKalmanState();

        // check default value
        assertNull(state.getCovariance());
        assertFalse(state.getCovariance(new Matrix(1, 1)));

        // set new value
        final var covariance1 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        state.setCovariance(covariance1);

        // check
        final var covariance2 = new Matrix(1, 1);
        assertTrue(state.getCovariance(covariance2));
        final var covariance3 = state.getCovariance();

        assertEquals(covariance1, covariance2);
        assertEquals(covariance1, covariance3);
    }

    @Test
    void testCopyToWhenInputHasValuesAndOutputDoesNotHaveValues() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);

        assertEquals(state1.getEstimation(), estimation);
        assertEquals(state1.getCovariance(), covariance);

        final var state2 = new GNSSKalmanState();
        state1.copyTo(state2);


        assertEquals(state2.getEstimation(), estimation);
        assertEquals(state2.getCovariance(), covariance);
    }

    @Test
    void testCopyToWhenInputHasNoValuesAndOutputHasValues() throws WrongSizeException {
        final var state1 = new GNSSKalmanState();

        assertNull(state1.getEstimation());
        assertNull(state1.getCovariance());

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);

        final var state2 = new GNSSKalmanState(estimation, covariance);
        state1.copyTo(state2);

        assertNull(state2.getEstimation());
        assertNull(state2.getCovariance());
    }

    @Test
    void testCopyToWhenInputHasNoValuesAndOutputDoesNotHaveValues() {
        final var state1 = new GNSSKalmanState();

        assertNull(state1.getEstimation());
        assertNull(state1.getCovariance());

        final var state2 = new GNSSKalmanState();
        state1.copyTo(state2);

        assertNull(state2.getEstimation());
        assertNull(state2.getCovariance());
    }

    @Test
    void testCopyToWhenBothInputAndOutputHaveValues() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation1 = new GNSSEstimation(x1, y1, z1, vx1, vy1, vz1, clockOffset1, clockDrift1);
        final var covariance1 = new Matrix(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation1, covariance1);

        final var x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation2 = new GNSSEstimation(x2, y2, z2, vx2, vy2, vz2, clockOffset2, clockDrift2);
        final var covariance2 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state2 = new GNSSKalmanState(estimation2, covariance2);

        state1.copyTo(state2);

        assertEquals(state2.getEstimation(), estimation1);
        assertEquals(state2.getCovariance(), covariance1);
    }

    @Test
    void testCopyFrom() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation1 = new GNSSEstimation(x1, y1, z1, vx1, vy1, vz1, clockOffset1, clockDrift1);
        final var covariance1 = new Matrix(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation1, covariance1);

        final var x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation2 = new GNSSEstimation(x2, y2, z2, vx2, vy2, vz2, clockOffset2, clockDrift2);
        final var covariance2 = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state2 = new GNSSKalmanState(estimation2, covariance2);

        state2.copyFrom(state1);

        assertEquals(state2.getEstimation(), estimation1);
        assertEquals(state2.getCovariance(), covariance1);
    }

    @Test
    void testHashCode() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);
        final var state2 = new GNSSKalmanState(estimation, covariance);
        final var state3 = new GNSSKalmanState();

        assertEquals(state1.hashCode(), state2.hashCode());
        assertNotEquals(state1.hashCode(), state3.hashCode());
    }

    @Test
    void testEquals() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);
        final var state2 = new GNSSKalmanState(estimation, covariance);
        final var state3 = new GNSSKalmanState();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(state1.equals((Object) state1));
        //noinspection EqualsWithItself
        assertTrue(state1.equals(state1));
        assertTrue(state1.equals(state2));
        assertFalse(state1.equals(state3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(state1.equals((Object) null));
        assertFalse(state1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), state1);
    }

    @Test
    void testEqualsWithThreshold() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);
        final var state2 = new GNSSKalmanState(estimation, covariance);
        final var state3 = new GNSSKalmanState();

        assertTrue(state1.equals(state1, THRESHOLD));
        assertTrue(state1.equals(state2, THRESHOLD));
        assertFalse(state1.equals(state3, THRESHOLD));
        assertFalse(state1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws WrongSizeException, CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);

        final var state2 = state1.clone();

        assertEquals(state1, state2);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var covariance = Matrix.identity(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        final var state1 = new GNSSKalmanState(estimation, covariance);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(state1);
        final var state2 = SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(state1, state2);
        assertEquals(state1, state2);
    }
}
