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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class INSLooselyCoupledKalmanInitializerTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int NUM_PARAMS = INSLooselyCoupledKalmanInitializer.NUM_PARAMS;

    @Test
    public void testInitialize() throws WrongSizeException {
        final INSLooselyCoupledKalmanInitializerConfig config = generateConfig();

        final Matrix expected = Matrix.diagonal(new double[] {
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty()
        });

        final Matrix result1 = new Matrix(NUM_PARAMS, NUM_PARAMS);
        INSLooselyCoupledKalmanInitializer.initialize(config, result1);
        final Matrix result2 = new Matrix(1, 1);
        INSLooselyCoupledKalmanInitializer.initialize(config, result2);
        final Matrix result3 = INSLooselyCoupledKalmanInitializer.initialize(config);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
        assertEquals(expected, result3);
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        return new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty);
    }
}
