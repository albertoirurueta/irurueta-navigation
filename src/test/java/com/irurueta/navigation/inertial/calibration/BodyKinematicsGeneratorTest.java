/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class BodyKinematicsGeneratorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ACCEL_QUANT_RESIDUAL = -5e-3;
    private static final double MAX_ACCEL_QUANT_RESIDUAL = 5e-3;

    private static final double MIN_GYRO_QUANT_RESIDUAL = -5e-5;
    private static final double MAX_GYRO_QUANT_RESIDUAL = 5e-5;

    private static final int NUM = 10;

    private static final int SAMPLES = 100000;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    @Test
    public void testGenerateSingleTimeIntervalQuantizedAndWithNoise()
            throws WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = getAccelQuantLevel();
        final double gyroQuantLevel = getGyroQuantLevel();

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final double[] oldQuantizationResiduals = new double[6];
        for (int i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (int i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final double[] quantizationResiduals1 = new double[6];
        final BodyKinematics result1 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);
        final double[] quantizationResiduals2 = new double[6];
        final BodyKinematics result2 = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                oldQuantizationResiduals, quantizationResiduals2);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics result3 = new BodyKinematics();
        final double[] quantizationResiduals3 = new double[6];
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors,
                random, oldQuantizationResiduals, result3, quantizationResiduals3);

        final double[] quantizationResiduals4 = new double[6];
        final BodyKinematics result4 = BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors, random, oldQuantizationResiduals,
                quantizationResiduals4);

        final double[] expectedQuantizationResiduals = new double[6];
        final BodyKinematics expected = generate(TIME_INTERVAL_SECONDS, trueKinematics,
                errors, random, oldQuantizationResiduals, expectedQuantizationResiduals);


        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1,
                ABSOLUTE_ERROR);

        assertTrue(expected.equals(result2, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals2,
                ABSOLUTE_ERROR);

        assertTrue(expected.equals(result3, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals3,
                ABSOLUTE_ERROR);

        assertTrue(expected.equals(result4, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals4,
                ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, new double[1], result1,
                    quantizationResiduals1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, oldQuantizationResiduals,
                    result1, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        BodyKinematics result = null;
        try {
            result = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, new double[1],
                    quantizationResiduals2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, oldQuantizationResiduals,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        try {
            BodyKinematicsGenerator.generate(timeInterval,
                    trueKinematics, errors, random, new double[1], result3,
                    quantizationResiduals3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            BodyKinematicsGenerator.generate(timeInterval,
                    trueKinematics, errors, random, oldQuantizationResiduals,
                    result3, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        result = null;
        try {
            result = BodyKinematicsGenerator.generate(timeInterval,
                    trueKinematics, errors, random, new double[1],
                    quantizationResiduals4);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = BodyKinematicsGenerator.generate(timeInterval,
                    trueKinematics, errors, random, oldQuantizationResiduals,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);
    }

    @Test
    public void testGenerateSingleNoTimeIntervalQuantizedAndWithNoise()
            throws WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = getAccelQuantLevel();
        final double gyroQuantLevel = getGyroQuantLevel();

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final double[] oldQuantizationResiduals = new double[6];
        for (int i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (int i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final double[] quantizationResiduals1 = new double[6];
        final BodyKinematics result1 = new BodyKinematics();
        BodyKinematicsGenerator.generate(0.0,
                trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);

        final double[] expectedQuantizationResiduals = new double[6];
        final BodyKinematics expected = generate(0.0, trueKinematics,
                errors, random, oldQuantizationResiduals, expectedQuantizationResiduals);


        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1,
                ABSOLUTE_ERROR);

        verifyZeroInteractions(random);
    }

    @Test
    public void testGenerateSingleTimeIntervalQuantizedAndWithoutNoise()
            throws WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = getAccelQuantLevel();
        final double gyroQuantLevel = getGyroQuantLevel();

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final double[] oldQuantizationResiduals = new double[6];
        for (int i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (int i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final double[] quantizationResiduals1 = new double[6];
        final BodyKinematics result1 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);

        // When no noise is present, is equivalent to zero time interval
        final double[] expectedQuantizationResiduals = new double[6];
        final BodyKinematics expected = generate(0.0, trueKinematics,
                errors, random, oldQuantizationResiduals, expectedQuantizationResiduals);


        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1,
                ABSOLUTE_ERROR);

        verify(random, atLeast(6)).nextGaussian();
    }

    @Test
    public void testGenerateSingleTimeIntervalNotQuantizedAndWithNoise()
            throws WrongSizeException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel1 = 0.0;
        final double gyroQuantLevel1 = 0.0;

        final IMUErrors errors1 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel1, gyroQuantLevel1);

        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final double[] oldQuantizationResiduals = new double[6];
        for (int i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (int i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final double[] quantizationResiduals1 = new double[6];
        final BodyKinematics result1 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors1, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);


        final double accelQuantLevel2 = getAccelQuantLevel();
        final double gyroQuantLevel2 = getGyroQuantLevel();
        final IMUErrors errors2 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel2, gyroQuantLevel2);

        // when no old quantization residuals are provided, quantization gets disabled
        final double[] quantizationResiduals2 = new double[6];
        final BodyKinematics result2 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors2, random, null,
                result2, quantizationResiduals2);

        final BodyKinematics result3 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors1, random, oldQuantizationResiduals,
                result3, null);

        final BodyKinematics result4 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors1, random, result4);

        final BodyKinematics result5 = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS, trueKinematics, errors1, random);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics result6 = new BodyKinematics();
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1,
                random, result6);

        final BodyKinematics result7 = BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors1, random);

        final double[] expectedQuantizationResiduals = new double[6];
        final BodyKinematics expected = generate(TIME_INTERVAL_SECONDS, trueKinematics,
                errors1, random, oldQuantizationResiduals, expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1,
                ABSOLUTE_ERROR);

        assertTrue(expected.equals(result2, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals2,
                ABSOLUTE_ERROR);

        assertTrue(expected.equals(result3, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result4, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result5, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result6, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result7, ABSOLUTE_ERROR));

        assertArrayEquals(quantizationResiduals1, new double[6], 0.0);
        assertArrayEquals(quantizationResiduals2, new double[6], 0.0);
    }

    @Test
    public void testGenerateSingleNoTimeIntervalNoQuantizationAndNoNoise()
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final double[] oldQuantizationResiduals = new double[6];
        for (int i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (int i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(
                    MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final double[] quantizationResiduals1 = new double[6];
        final BodyKinematics result1 = new BodyKinematics();
        BodyKinematicsGenerator.generate(0.0,
                trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);

        final double[] expectedQuantizationResiduals = new double[6];
        final BodyKinematics expected = generate(0.0, trueKinematics,
                errors, random, oldQuantizationResiduals, expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1,
                ABSOLUTE_ERROR);

        // Because we follow the model
        // uf = ba + (I + Ma)*f + noise
        // ug = bg + (I + Mg)*g + Gg*f + noise

        // Neglecting noise terms,
        // then true values can be recovered as:
        // f = (I + Ma)^-1*(uf - ba)
        // g = (I + Mg)^-1*(ug - bg - Gg*f)

        // Hence:
        final Matrix uf = result1.asSpecificForceMatrix();
        final Matrix ug = result1.asAngularRateMatrix();

        final Matrix identity = Matrix.identity(3, 3);
        final Matrix f = Utils.inverse(identity.addAndReturnNew(ma))
                .multiplyAndReturnNew(uf.subtractAndReturnNew(ba));

        assertTrue(f.equals(trueKinematics.asSpecificForceMatrix(), ABSOLUTE_ERROR));

        final Matrix g = Utils.inverse(identity.addAndReturnNew(mg))
                .multiplyAndReturnNew(ug.subtractAndReturnNew(bg)
                        .subtractAndReturnNew(gg.multiplyAndReturnNew(f)));

        assertTrue(g.equals(trueKinematics.asAngularRateMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGenerateCollection() throws WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = getAccelQuantLevel();
        final double gyroQuantLevel = getGyroQuantLevel();

        final IMUErrors errors1 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
        final IMUErrors errors2 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, 0.0, 0.0);


        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] oldQuantizationResiduals = new double[6];
        final double[] quantizationResiduals = new double[6];

        final List<BodyKinematics> trueKinematics = new ArrayList<>();
        final List<BodyKinematics> expected = new ArrayList<>();
        for (int i = 0; i < NUM; i++) {
            final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                    MAX_ACCELEROMETER_VALUE);
            final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                    MAX_ACCELEROMETER_VALUE);
            final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                    MAX_ACCELEROMETER_VALUE);
            final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

            trueKinematics.add(new BodyKinematics(fx, fy, fz,
                    omegaX, omegaY, omegaZ));

            expected.add(generate(TIME_INTERVAL_SECONDS, trueKinematics.get(i),
                    errors2, random, oldQuantizationResiduals, quantizationResiduals));
        }

        final List<BodyKinematics> result1 = new ArrayList<>();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                errors1, random, result1);
        final Collection<BodyKinematics> result2 = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS, trueKinematics, errors1, random);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final List<BodyKinematics> result3 = new ArrayList<>();
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1,
                random, result3);

        final Collection<BodyKinematics> result4 = BodyKinematicsGenerator.generate(
                timeInterval, trueKinematics, errors1, random);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
        assertEquals(expected, result3);
        assertEquals(expected, result4);
    }

    @Test
    public void testEstimateNoiseRootPSDs() throws WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);

        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final BodyKinematics result = new BodyKinematics();
        double avgFx = 0.0;
        double avgFy = 0.0;
        double avgFz = 0.0;
        double avgOmegaX = 0.0;
        double avgOmegaY = 0.0;
        double avgOmegaZ = 0.0;
        double varFx = 0.0;
        double varFy = 0.0;
        double varFz = 0.0;
        double varOmegaX = 0.0;
        double varOmegaY = 0.0;
        double varOmegaZ = 0.0;
        for (int i = 0, j = 1; i < SAMPLES; i++, j++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random, result);

            avgFx = avgFx * (double) i / (double) j + result.getFx() / j;
            avgFy = avgFy * (double) i / (double) j + result.getFy() / j;
            avgFz = avgFz * (double) i / (double) j + result.getFz() / j;

            avgOmegaX = avgOmegaX * (double) i / (double) j + result.getAngularRateX() / j;
            avgOmegaY = avgOmegaY * (double) i / (double) j + result.getAngularRateY() / j;
            avgOmegaZ = avgOmegaZ * (double) i / (double) j + result.getAngularRateZ() / j;

            double diff = result.getFx() - avgFx;
            double diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = result.getFy() - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = result.getFz() - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateX() - avgOmegaX;
            diff2 = diff * diff;
            varOmegaX = varOmegaX * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateY() - avgOmegaY;
            diff2 = diff * diff;
            varOmegaY = varOmegaY * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateZ() - avgOmegaZ;
            diff2 = diff * diff;
            varOmegaZ = varOmegaZ * (double) i / (double) j + diff2 / j;
        }

        final double rootPsdFx = Math.sqrt(varFx  * TIME_INTERVAL_SECONDS);
        final double rootPsdFy = Math.sqrt(varFy * TIME_INTERVAL_SECONDS);
        final double rootPsdFz = Math.sqrt(varFz * TIME_INTERVAL_SECONDS);
        final double rootPsdOmegaX = Math.sqrt(varOmegaX * TIME_INTERVAL_SECONDS);
        final double rootPsdOmegaY = Math.sqrt(varOmegaY * TIME_INTERVAL_SECONDS);
        final double rootPsdOmegaZ = Math.sqrt(varOmegaZ * TIME_INTERVAL_SECONDS);

        assertEquals(rootPsdFx, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);

        assertEquals(rootPsdOmegaX, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaY, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaZ, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
    }

    private BodyKinematics generate(final double timeInterval,
                                    final BodyKinematics trueKinematics,
                                    final IMUErrors errors,
                                    final Random random,
                                    final double[] oldQuantizationResiduals,
                                    final double[] quantizationResiduals) throws WrongSizeException {

        final Matrix accelNoise = new Matrix(3, 1);
        final Matrix gyroNoise = new Matrix(3, 1);
        if (timeInterval > 0.0) {
            for (int i = 0; i < 3; i++) {
                accelNoise.setElementAtIndex(i, random.nextGaussian()
                        * errors.getAccelerometerNoiseRootPSD()
                        / Math.sqrt(timeInterval));
                gyroNoise.setElementAtIndex(i, random.nextGaussian()
                        * errors.getGyroNoiseRootPSD() / Math.sqrt(timeInterval));
            }
        }

        final Matrix trueFibb = trueKinematics.asSpecificForceMatrix();
        final Matrix trueOmegaIbb = trueKinematics.asAngularRateMatrix();

        final Matrix identity = Matrix.identity(3, 3);

        final Matrix ba = errors.getAccelerometerBiasesAsMatrix();
        final Matrix ma = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();

        final Matrix bg = errors.getGyroBiasesAsMatrix();
        final Matrix mg = errors.getGyroScaleFactorAndCrossCouplingErrors();
        final Matrix gg = errors.getGyroGDependentBiases();

        final Matrix uqFibb = ba.addAndReturnNew((identity.addAndReturnNew(ma))
                .multiplyAndReturnNew(trueFibb))
                .addAndReturnNew(accelNoise);
        final Matrix uqOmegaibb = bg.addAndReturnNew((identity.addAndReturnNew(mg))
                .multiplyAndReturnNew(trueOmegaIbb))
                .addAndReturnNew(gg.multiplyAndReturnNew(trueFibb))
                .addAndReturnNew(gyroNoise);

        final Matrix oldRes = Matrix.newFromArray(oldQuantizationResiduals);

        final double accelQuantLevel = errors.getAccelerometerQuantizationLevel();
        final Matrix measFibb;
        if (accelQuantLevel > 0.0) {
            final Matrix tmp = (uqFibb.addAndReturnNew(oldRes
                    .getSubmatrix(0, 0, 2, 0)))
                    .multiplyByScalarAndReturnNew(1.0 / accelQuantLevel);
            for (int i = 0; i < 3; i++) {
                tmp.setElementAtIndex(i, Math.round(tmp.getElementAtIndex(i)));
            }
            measFibb = tmp.multiplyByScalarAndReturnNew(accelQuantLevel);


            for (int i = 0; i < 3; i++) {
                quantizationResiduals[i] = uqFibb.getElementAtIndex(i)
                        + oldQuantizationResiduals[i] - measFibb.getElementAtIndex(i);
            }
        } else {
            measFibb = uqFibb;


            for (int i = 0; i < 3; i++) {
                quantizationResiduals[i] = 0.0;
            }
        }

        final double gyroQuantLevel = errors.getGyroQuantizationLevel();
        final Matrix measOmegaIbb;
        if (gyroQuantLevel > 0.0) {
            final Matrix tmp = (uqOmegaibb.addAndReturnNew(oldRes
                    .getSubmatrix(3, 0, 5, 0)))
                    .multiplyByScalarAndReturnNew(1.0 / gyroQuantLevel);
            for (int i = 0; i < 3; i++) {
                tmp.setElementAtIndex(i, Math.round(tmp.getElementAtIndex(i)));
            }
            measOmegaIbb = tmp.multiplyByScalarAndReturnNew(gyroQuantLevel);


            for (int i = 0, j = 3; i < 3; i++, j++) {
                quantizationResiduals[j] = uqOmegaibb.getElementAtIndex(i)
                        + oldQuantizationResiduals[j] - measOmegaIbb.getElementAtIndex(i);
            }
        } else {
            measOmegaIbb = uqOmegaibb;


            for (int i = 3; i < 6; i++) {
                quantizationResiduals[i] = 0.0;
            }
        }

        return new BodyKinematics(measFibb.getElementAtIndex(0),
                measFibb.getElementAtIndex(1), measFibb.getElementAtIndex(2),
                measOmegaIbb.getElementAtIndex(0), measOmegaIbb.getElementAtIndex(1),
                measOmegaIbb.getElementAtIndex(2));
    }

    private Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private Matrix generateMa() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private Matrix generateGg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        final double tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }

    private double getAccelQuantLevel() {
        return 1e-2;
    }

    private double getGyroQuantLevel() {
        return 2e-4;
    }
}
