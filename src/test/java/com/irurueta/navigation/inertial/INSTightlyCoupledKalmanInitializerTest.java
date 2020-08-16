package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class INSTightlyCoupledKalmanInitializerTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int NUM_PARAMS = INSTightlyCoupledKalmanInitializer.NUM_PARAMS;

    @Test
    public void testInitialize() throws WrongSizeException {
        final INSTightlyCoupledKalmanInitializerConfig config = generateConfig();

        final Matrix expected = Matrix.diagonal(new double[]{
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
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialClockOffsetUncertainty() * config.getInitialClockOffsetUncertainty(),
                config.getInitialClockDriftUncertainty() * config.getInitialClockDriftUncertainty()
        });

        final Matrix result1 = new Matrix(NUM_PARAMS, NUM_PARAMS);
        INSTightlyCoupledKalmanInitializer.initialize(config, result1);
        final Matrix result2 = new Matrix(1, 1);
        INSTightlyCoupledKalmanInitializer.initialize(config, result2);
        final Matrix result3 = INSTightlyCoupledKalmanInitializer.initialize(config);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
        assertEquals(expected, result3);
    }

    private static INSTightlyCoupledKalmanInitializerConfig generateConfig() {
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
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        return new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);

    }
}
