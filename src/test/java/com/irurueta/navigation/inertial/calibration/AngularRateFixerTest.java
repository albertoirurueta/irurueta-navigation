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

import static org.junit.Assert.*;

public class AngularRateFixerTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testConstructor() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        assertEquals(fixer.getBias(), new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        fixer.getBias(b);
        assertEquals(b, new Matrix(3, 1));

        assertArrayEquals(fixer.getBiasArray(), new double[3], 0.0);
        final double[] b2 = new double[3];
        fixer.getBiasArray(b2);
        assertArrayEquals(b2, new double[3], 0.0);
        assertEquals(fixer.getBiasX(), 0.0, 0.0);
        assertEquals(fixer.getBiasY(), 0.0, 0.0);
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        assertEquals(fixer.getCrossCouplingErrors(),
                new Matrix(3, 3));
        final Matrix m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(m, new Matrix(3, 3));

        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        assertEquals(fixer.getGDependantCrossBias(),
                new Matrix(3, 3));
        final Matrix g = new Matrix(3, 3);
        fixer.getGDependantCrossBias(g);
        assertEquals(g, new Matrix(3, 3));
    }

    @Test
    public void testGetSetBias() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        final Matrix b1 = fixer.getBias();
        final Matrix b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(b1, new Matrix(3, 1));
        assertEquals(b1, b2);

        // set new value
        final Matrix b3 = generateBg();
        fixer.setBias(b3);

        // check
        final Matrix b4 = fixer.getBias();
        final Matrix b5 = new Matrix(3, 1);
        fixer.getBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        try {
            fixer.setBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBiasArray() {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        final double[] b1 = fixer.getBiasArray();
        final double[] b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(b1, new double[3], 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final double[] b3 = generateBg().getBuffer();
        fixer.setBias(b3);

        // check
        final double[] b4 = fixer.getBiasArray();
        final double[] b5 = new double[3];
        fixer.getBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        try {
            fixer.getBiasArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBiasX() {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getBiasX(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    public void testGetSetBiasY() {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getBiasY(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double by = b.getElementAtIndex(1);
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    public void testGetSetBiasZ() {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double bz = b.getElementAtIndex(2);
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBias() {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        assertEquals(fixer.getBiasX(), 0.0, 0.0);
        assertEquals(fixer.getBiasY(), 0.0, 0.0);
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        final double by = b.getElementAtIndex(1);
        final double bz = b.getElementAtIndex(2);
        fixer.setBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
        assertEquals(by, fixer.getBiasY(), 0.0);
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetCrossCouplingErrors() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        final Matrix m1 = fixer.getCrossCouplingErrors();
        final Matrix m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(m1, new Matrix(3, 3));
        assertEquals(m1, m2);

        // set new values
        final Matrix m3 = generateMg();
        final double sx = m3.getElementAt(0, 0);
        final double sy = m3.getElementAt(1, 1);
        final double sz = m3.getElementAt(2, 2);
        final double mxy = m3.getElementAt(0, 1);
        final double mxz = m3.getElementAt(0, 2);
        final double myx = m3.getElementAt(1, 0);
        final double myz = m3.getElementAt(1, 2);
        final double mzx = m3.getElementAt(2, 0);
        final double mzy = m3.getElementAt(2, 1);

        fixer.setCrossCouplingErrors(m3);

        // check
        final Matrix m4 = fixer.getCrossCouplingErrors();
        final Matrix m5 = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getSx(), 0.0);
        assertEquals(sy, fixer.getSy(), 0.0);
        assertEquals(sz, fixer.getSz(), 0.0);
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);

        // Force IllegalArgumentException
        try {
            fixer.setCrossCouplingErrors(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setCrossCouplingErrors(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force AlgebraException
        try {
            final Matrix wrong = Matrix.identity(3, 3);
            wrong.multiplyByScalar(-1.0);
            fixer.setCrossCouplingErrors(wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
    }

    @Test
    public void testGetSetSx() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getSx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    public void testGetSetSy() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getSy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    public void testGetSetSz() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    public void testGetSetMxy() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMxy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    public void testGetSetMxz() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMxz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    public void testGetSetMyx() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMyx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    public void testGetSetMyz() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMyz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    public void testGetSetMzx() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMzx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    public void testGetSetMzy() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default value
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testSetScalingFactors() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMg();
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);

        fixer.setScalingFactors(sx, sy, sz);

        // check
        assertEquals(fixer.getSx(), sx, 0.0);
        assertEquals(fixer.getSy(), sy, 0.0);
        assertEquals(fixer.getSz(), sz, 0.0);
    }

    @Test
    public void testSetCrossCouplingErrors() throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMg();
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testSetScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMg();
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setScalingFactorsAndCrossCouplingErrors(sx, sy, sz,
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(fixer.getSx(), sx, 0.0);
        assertEquals(fixer.getSy(), sy, 0.0);
        assertEquals(fixer.getSz(), sz, 0.0);
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testGetSetGDependantCrossBias() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        // check default values
        final Matrix g1 = fixer.getGDependantCrossBias();
        final Matrix g2 = new Matrix(1, 1);
        fixer.getGDependantCrossBias(g2);

        assertEquals(g1, new Matrix(3, 3));
        assertEquals(g1, g2);

        // set new values
        final Matrix g3 = generateGg();

        fixer.setGDependantCrossBias(g3);

        // check
        final Matrix g4 = fixer.getGDependantCrossBias();
        final Matrix g5 = new Matrix(3, 3);
        fixer.getGDependantCrossBias(g5);

        assertEquals(g3, g4);
        assertEquals(g3, g5);

        // Force IllegalArgumentException
        try {
            fixer.setGDependantCrossBias(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setGDependantCrossBias(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix1() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double[] measuredAngularRate = measuredKinematics
                .asAngularRateArray();
        final double[] trueF = trueKinematics.asSpecificForceArray();

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], trueF, result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, new double[1], result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix2() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), trueF, result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), trueF, result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(1, 1), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(3, 3), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix3() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final Matrix result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final Matrix result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), trueF, result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), trueF, result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(1, 1), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(3, 3), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix4() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, result1);
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix5() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final Matrix result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final Matrix result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result1);
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix6() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double[] measuredAngularRate = measuredKinematics
                .asAngularRateArray();
        final double[] trueF = trueKinematics.asSpecificForceArray();

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], trueF, bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, new double[1], bg, mg, gg,
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(1, 1), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(3, 3), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(1, 3), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(3, 1), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, gg, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix7() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), trueF,
                    bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), trueF,
                    bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(1, 1), bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(3, 3), bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(1, 1), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(3, 3), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(1, 3), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(3, 1), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, gg, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix8() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final Matrix result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), trueF,
                    bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), trueF,
                    bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(1, 1), bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate,
                    new Matrix(3, 3), bg, mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(1, 1), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    new Matrix(3, 3), mg, gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(1, 3), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, new Matrix(3, 1), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, gg, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredAngularRate, trueF,
                    bg, mg, gg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix9() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, result);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(1, 3), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(3, 1), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, gg, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix10() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final Matrix result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, result);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(1, 3), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(3, 1), gg, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, gg, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, gg, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix11() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double myx = mg.getElementAt(1, 0);
        final double mzx = mg.getElementAt(2, 0);
        final double mxy = mg.getElementAt(0, 1);
        final double sy = mg.getElementAt(1, 1);
        final double mzy = mg.getElementAt(2, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myz = mg.getElementAt(1, 2);
        final double sz = mg.getElementAt(2, 2);

        final double g11 = gg.getElementAt(0, 0);
        final double g21 = gg.getElementAt(1, 0);
        final double g31 = gg.getElementAt(2, 0);
        final double g12 = gg.getElementAt(0, 1);
        final double g22 = gg.getElementAt(1, 1);
        final double g32 = gg.getElementAt(2, 1);
        final double g13 = gg.getElementAt(0, 2);
        final double g23 = gg.getElementAt(1, 2);
        final double g33 = gg.getElementAt(2, 2);

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33, result);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    g11, g21, g31, g12, g22, g32, g13, g23, g33,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix12() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double myx = mg.getElementAt(1, 0);
        final double mzx = mg.getElementAt(2, 0);
        final double mxy = mg.getElementAt(0, 1);
        final double sy = mg.getElementAt(1, 1);
        final double mzy = mg.getElementAt(2, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myz = mg.getElementAt(1, 2);
        final double sz = mg.getElementAt(2, 2);

        final double g11 = gg.getElementAt(0, 0);
        final double g21 = gg.getElementAt(1, 0);
        final double g31 = gg.getElementAt(2, 0);
        final double g12 = gg.getElementAt(0, 1);
        final double g22 = gg.getElementAt(1, 1);
        final double g32 = gg.getElementAt(2, 1);
        final double g13 = gg.getElementAt(0, 2);
        final double g23 = gg.getElementAt(1, 2);
        final double g33 = gg.getElementAt(2, 2);

        final Matrix result = new Matrix(BodyKinematics.COMPONENTS,
                1);
        fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33, result);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    g11, g21, g31, g12, g22, g32, g13, g23, g33,
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    g11, g21, g31, g12, g22, g32, g13, g23, g33,
                    new Matrix(3,3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew1() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double[] measuredAngularRate = measuredKinematics
                .asAngularRateArray();
        final double[] trueF = trueKinematics.asSpecificForceArray();

        final double[] result1 = fixer.fixAndReturnNew(
                measuredAngularRate, trueF);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new double[1], trueF);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew2() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final double[] result1 = fixer.fixAndReturnNew(
                measuredAngularRate, trueF);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(
                    new Matrix(1, 1), trueF);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    new Matrix(3, 3), trueF);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix1() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final Matrix result1 = fixer.fixAndReturnNewMatrix(
                measuredAngularRate, trueF);
        final Matrix result2 = fixer.fixAndReturnNewMatrix(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNewMatrix(
                    new Matrix(1, 1), trueF);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    new Matrix(3, 3), trueF);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRate, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew3() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] result1 = fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix2() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final Matrix result1 = fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz);
        final Matrix result2 = fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew4() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double[] measuredAngularRate = measuredKinematics
                .asAngularRateArray();
        final double[] trueF = trueKinematics.asSpecificForceArray();

        final double[] result = fixer.fixAndReturnNew(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new double[1], trueF,
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new double[1], bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew5() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final double[] result = fixer.fixAndReturnNew(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(
                    new Matrix(1, 1), trueF, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    new Matrix(3, 3), trueF, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new Matrix(1, 1), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    new Matrix(3, 3), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredAngularRate,
                    trueF, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix3() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final Matrix measuredAngularRate = measuredKinematics
                .asAngularRateMatrix();
        final Matrix trueF = trueKinematics.asSpecificForceMatrix();

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNewMatrix(
                    new Matrix(1, 1), trueF,
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    new Matrix(3, 3), trueF,
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRate, new Matrix(1, 1),
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    new Matrix(3, 3), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(measuredAngularRate,
                    trueF, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew6() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] result = fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix4() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(
                    measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                    trueFx, trueFy, trueFz, bgx, bgy, bgz,
                    mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew7() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double myx = mg.getElementAt(1, 0);
        final double mzx = mg.getElementAt(2, 0);
        final double mxy = mg.getElementAt(0, 1);
        final double sy = mg.getElementAt(1, 1);
        final double mzy = mg.getElementAt(2, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myz = mg.getElementAt(1, 2);
        final double sz = mg.getElementAt(2, 2);

        final double g11 = gg.getElementAt(0, 0);
        final double g21 = gg.getElementAt(1, 0);
        final double g31 = gg.getElementAt(2, 0);
        final double g12 = gg.getElementAt(0, 1);
        final double g22 = gg.getElementAt(1, 1);
        final double g32 = gg.getElementAt(2, 1);
        final double g13 = gg.getElementAt(0, 2);
        final double g23 = gg.getElementAt(1, 2);
        final double g33 = gg.getElementAt(2, 2);

        final double[] result = fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix5() throws AlgebraException {
        final AngularRateFixer fixer = new AngularRateFixer();

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

        final double measuredAngularRateX = measuredKinematics.getAngularRateX();
        final double measuredAngularRateY = measuredKinematics.getAngularRateY();
        final double measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final double trueFx = trueKinematics.getFx();
        final double trueFy = trueKinematics.getFy();
        final double trueFz = trueKinematics.getFz();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double myx = mg.getElementAt(1, 0);
        final double mzx = mg.getElementAt(2, 0);
        final double mxy = mg.getElementAt(0, 1);
        final double sy = mg.getElementAt(1, 1);
        final double mzy = mg.getElementAt(2, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myz = mg.getElementAt(1, 2);
        final double sz = mg.getElementAt(2, 2);

        final double g11 = gg.getElementAt(0, 0);
        final double g21 = gg.getElementAt(1, 0);
        final double g31 = gg.getElementAt(2, 0);
        final double g12 = gg.getElementAt(0, 1);
        final double g22 = gg.getElementAt(1, 1);
        final double g32 = gg.getElementAt(2, 1);
        final double g13 = gg.getElementAt(0, 2);
        final double g23 = gg.getElementAt(1, 2);
        final double g33 = gg.getElementAt(2, 2);

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33);

        // check
        assertEquals(result.getElementAtIndex(0), omegaX, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), omegaY, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), omegaZ, ABSOLUTE_ERROR);
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
