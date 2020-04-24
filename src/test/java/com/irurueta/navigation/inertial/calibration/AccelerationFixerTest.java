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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class AccelerationFixerTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testFix1() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double[] measuredF = measuredKinematics
                .asSpecificForceArray();

        final double[] result = new double[BodyKinematics.COMPONENTS];
        AccelerationFixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fix(new double[1], ba, ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, new Matrix(1, 1),
                    ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, new Matrix(3, 3),
                    ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, ba,
                    new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, ba,
                    new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, ba, ma, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix2() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final Matrix measuredF = measuredKinematics
                .asSpecificForceMatrix();

        final double[] result = new double[BodyKinematics.COMPONENTS];
        AccelerationFixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fix(new Matrix(1, 1), ba, ma,
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(new Matrix(3, 3), ba, ma,
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix3() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final Matrix measuredF = measuredKinematics
                .asSpecificForceMatrix();

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        AccelerationFixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fix(measuredF, ba, ma,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredF, ba, ma,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix4() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double[] result = new double[BodyKinematics.COMPONENTS];
        AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix5() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, ma,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, ma,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix6() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        final double[] result = new double[BodyKinematics.COMPONENTS];
        AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix7() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                    mzx, mzy, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                    mzx, mzy, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew1() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double[] measuredF = measuredKinematics
                .asSpecificForceArray();

        final double[] result = AccelerationFixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fixAndReturnNew(new double[1], ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fixAndReturnNew(measuredF,
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fixAndReturnNew(measuredF,
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fixAndReturnNew(measuredF, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fixAndReturnNew(measuredF, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew2() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final Matrix measuredF = measuredKinematics
                .asSpecificForceMatrix();

        final double[] result = AccelerationFixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            AccelerationFixer.fixAndReturnNew(
                    new Matrix(1, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            AccelerationFixer.fixAndReturnNew(
                    new Matrix(3, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix3() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final Matrix measuredF = measuredKinematics
                .asSpecificForceMatrix();

        final Matrix result = AccelerationFixer.fixAndReturnNewMatrix(
                measuredF, ba, ma);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew4() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double[] result = AccelerationFixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix5() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Matrix result = AccelerationFixer.fixAndReturnNewMatrix(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew6() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        final double[] result = AccelerationFixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew7() throws AlgebraException {
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

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final double measuredFx = measuredKinematics.getFx();
        final double measuredFy = measuredKinematics.getFy();
        final double measuredFz = measuredKinematics.getFz();

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        final Matrix result = AccelerationFixer.fixAndReturnNewMatrix(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
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
}
