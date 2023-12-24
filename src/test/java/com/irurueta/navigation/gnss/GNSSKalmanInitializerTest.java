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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class GNSSKalmanInitializerTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    @Test
    public void testInitialize() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Matrix covariance = Matrix.diagonal(new double[]{
                initialPositionUncertainty * initialPositionUncertainty,
                initialPositionUncertainty * initialPositionUncertainty,
                initialPositionUncertainty * initialPositionUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialClockOffsetUncertainty * initialClockOffsetUncertainty,
                initialClockDriftUncertainty * initialClockDriftUncertainty
        });

        final GNSSKalmanState expected = new GNSSKalmanState(estimation, covariance);

        final GNSSKalmanConfig config = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final GNSSKalmanState result1 = new GNSSKalmanState();
        GNSSKalmanInitializer.initialize(estimation, config, result1);

        final GNSSKalmanState result2 = GNSSKalmanInitializer.initialize(estimation, config);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
    }
}
