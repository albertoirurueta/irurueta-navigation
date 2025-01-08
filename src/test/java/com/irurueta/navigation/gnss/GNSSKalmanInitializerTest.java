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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class GNSSKalmanInitializerTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    @Test
    void testInitialize() {
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

        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var covariance = Matrix.diagonal(new double[]{
                initialPositionUncertainty * initialPositionUncertainty,
                initialPositionUncertainty * initialPositionUncertainty,
                initialPositionUncertainty * initialPositionUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialVelocityUncertainty * initialVelocityUncertainty,
                initialClockOffsetUncertainty * initialClockOffsetUncertainty,
                initialClockDriftUncertainty * initialClockDriftUncertainty
        });

        final var expected = new GNSSKalmanState(estimation, covariance);

        final var config = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final var result1 = new GNSSKalmanState();
        GNSSKalmanInitializer.initialize(estimation, config, result1);

        final var result2 = GNSSKalmanInitializer.initialize(estimation, config);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
    }
}
