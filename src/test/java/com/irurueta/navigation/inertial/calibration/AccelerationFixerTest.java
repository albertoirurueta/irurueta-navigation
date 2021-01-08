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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

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
    public void testConstructor() throws WrongSizeException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final AccelerationTriad triad1 = fixer.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        fixer.getBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(fixer.getCrossCouplingErrors(),
                new Matrix(3, 3));
        final Matrix m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(m, new Matrix(3, 3));

        final Acceleration bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());
        final Acceleration bx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasXAsAcceleration(bx2);
        assertEquals(bx1, bx2);
        final Acceleration by1 = fixer.getBiasYAsAcceleration();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());
        final Acceleration by2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasYAsAcceleration(by2);
        assertEquals(by1, by2);
        final Acceleration bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());
        final Acceleration bz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasZAsAcceleration(bz2);
        assertEquals(bz1, bz2);

        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);
    }

    @Test
    public void testGetSetBias() throws WrongSizeException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final Matrix b1 = fixer.getBias();
        final Matrix b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(b1, new Matrix(3, 1));
        assertEquals(b1, b2);

        // set new value
        final Matrix b3 = generateBa();
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
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final double[] b1 = fixer.getBiasArray();
        final double[] b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(b1, new double[3], 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final double[] b3 = generateBa().getBuffer();
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
    public void testGetSetBiasTriad() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final AccelerationTriad triad1 = fixer.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new value
        final AccelerationTriad triad2 = new AccelerationTriad();
        triad2.setValueCoordinates(generateBa());
        fixer.setBias(triad2);

        // check
        final AccelerationTriad triad3 = fixer.getBiasAsTriad();
        final AccelerationTriad triad4 = new AccelerationTriad();
        fixer.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetBiasX() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getBiasX(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    public void testGetSetBiasY() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getBiasY(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double by = b.getElementAtIndex(1);
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    public void testGetSetBiasZ() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double bz = b.getElementAtIndex(2);
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBias1() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default values
        assertEquals(fixer.getBiasX(), 0.0, 0.0);
        assertEquals(fixer.getBiasY(), 0.0, 0.0);
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix b = generateBa();
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
    public void testGetSetBiasXAsAcceleration() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final Acceleration bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        final Acceleration bx2 = new Acceleration(
                bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasX(bx2);

        // check
        final Acceleration bx3 = fixer.getBiasXAsAcceleration();
        final Acceleration bx4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasXAsAcceleration(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetBiasYAsAcceleration() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final Acceleration by1 = fixer.getBiasYAsAcceleration();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double by = b.getElementAtIndex(1);
        final Acceleration by2 = new Acceleration(
                by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasY(by2);

        // check
        final Acceleration by3 = fixer.getBiasYAsAcceleration();
        final Acceleration by4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasYAsAcceleration(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetBiasZAsAcceleration() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        final Acceleration bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double bz = b.getElementAtIndex(2);
        final Acceleration bz2 = new Acceleration(
                bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasZ(bz2);

        // check
        final Acceleration bz3 = fixer.getBiasZAsAcceleration();
        final Acceleration bz4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasZAsAcceleration(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testGetSetBias2() {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default values
        final Acceleration bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        final Acceleration by1 = fixer.getBiasYAsAcceleration();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        final Acceleration bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new values
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        final double by = b.getElementAtIndex(1);
        final double bz = b.getElementAtIndex(2);
        final Acceleration bx2 = new Acceleration(
                bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration by2 = new Acceleration(
                by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bz2 = new Acceleration(
                bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBias(bx2, by2, bz2);

        // check
        final Acceleration bx3 = fixer.getBiasXAsAcceleration();
        final Acceleration by3 = fixer.getBiasYAsAcceleration();
        final Acceleration bz3 = fixer.getBiasZAsAcceleration();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testGetSetCrossCouplingErrors() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default values
        final Matrix m1 = fixer.getCrossCouplingErrors();
        final Matrix m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(m1, new Matrix(3, 3));
        assertEquals(m1, m2);

        // set new values
        final Matrix m3 = generateMa();
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
    public void testGetSetSx() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getSx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    public void testGetSetSy() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getSy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    public void testGetSetSz() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    public void testGetSetMxy() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMxy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    public void testGetSetMxz() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMxz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    public void testGetSetMyx() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMyx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    public void testGetSetMyz() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMyz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    public void testGetSetMzx() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMzx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    public void testGetSetMzy() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default value
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testSetScalingFactors() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default values
        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMa();
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
    public void testSetCrossCouplingErrors() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        // check default values
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMa();
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
            throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        final Matrix m = generateMa();
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
    public void testFix1() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final AccelerationTriad measuredF = measuredKinematics
                .getSpecificForceTriad();

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredF, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix2() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final AccelerationTriad measuredF = measuredKinematics
                .getSpecificForceTriad();

        final Matrix result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredF, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredF, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix3() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final AccelerationTriad measuredF = measuredKinematics
                .getSpecificForceTriad();

        final AccelerationTriad result = new AccelerationTriad();
        fixer.fix(measuredF, result);

        // check
        assertEquals(result.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(result.getValueX(), fx, ABSOLUTE_ERROR);
        assertEquals(result.getValueY(), fy, ABSOLUTE_ERROR);
        assertEquals(result.getValueZ(), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix4() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Acceleration measuredFx = measuredKinematics.getSpecificForceX();
        final Acceleration measuredFy = measuredKinematics.getSpecificForceY();
        final Acceleration measuredFz = measuredKinematics.getSpecificForceZ();
        final AccelerationTriad result = new AccelerationTriad();
        fixer.fix(measuredFx, measuredFy, measuredFz, result);

        // check
        assertEquals(result.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(result.getValueX(), fx, ABSOLUTE_ERROR);
        assertEquals(result.getValueY(), fy, ABSOLUTE_ERROR);
        assertEquals(result.getValueZ(), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix5() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix6() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), result1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix7() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Matrix result1 = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        final Matrix result2 = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredF, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix8() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = new double[BodyKinematics.COMPONENTS];
        final double[] result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredFx, measuredFy, measuredFz, result1);
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix9() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Matrix result1 = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        final Matrix result2 = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredFx, measuredFy, measuredFz, result1);
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix10() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], ba, ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, new Matrix(1, 1),
                    ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, new Matrix(3, 3),
                    ma, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, ba,
                    new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, ba,
                    new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, ba, ma, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix11() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), ba, ma,
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), ba, ma,
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix12() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredF, ba, ma,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredF, ba, ma,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix13() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix14() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, ma,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, ma,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix15() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFix16() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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
        fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                    mzx, mzy, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredFx, measuredFy, measuredFz,
                    biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                    mzx, mzy, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew1() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final AccelerationTriad measuredF = measuredKinematics
                .getSpecificForceTriad();

        final AccelerationTriad result = fixer.fixAndReturnNew(measuredF);

        // check
        assertEquals(result.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(result.getValueX(), fx, ABSOLUTE_ERROR);
        assertEquals(result.getValueY(), fy, ABSOLUTE_ERROR);
        assertEquals(result.getValueZ(), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew2() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Acceleration measuredFx = measuredKinematics.getSpecificForceX();
        final Acceleration measuredFy = measuredKinematics.getSpecificForceY();
        final Acceleration measuredFz = measuredKinematics.getSpecificForceZ();

        final AccelerationTriad result = fixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz);

        // check
        assertEquals(result.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(result.getValueX(), fx, ABSOLUTE_ERROR);
        assertEquals(result.getValueY(), fy, ABSOLUTE_ERROR);
        assertEquals(result.getValueZ(), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew3() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = fixer.fixAndReturnNew(
                measuredF);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew4() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = fixer.fixAndReturnNew(
                measuredF);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix3() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Matrix result1 = fixer.fixAndReturnNewMatrix(
                measuredF);
        final Matrix result2 = fixer.fixAndReturnNewMatrix(
                measuredF, ba, ma);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew5() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final double[] result1 = fixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz);
        final double[] result2 = fixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], fx, ABSOLUTE_ERROR);
        assertEquals(result1[1], fy, ABSOLUTE_ERROR);
        assertEquals(result1[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix5() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

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

        final Matrix result1 = fixer.fixAndReturnNewMatrix(
                measuredFx, measuredFy, measuredFz);
        final Matrix result2 = fixer.fixAndReturnNewMatrix(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertEquals(result1, result2);
        assertEquals(result1.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result1.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew6() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final double[] result = fixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new double[1], ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredF,
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredF,
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredF, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(measuredF, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew7() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final double[] result = fixer.fixAndReturnNew(
                measuredF, ba, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(
                    new Matrix(1, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(
                    new Matrix(3, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix8() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredF, ba, ma);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew8() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final double[] result = fixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix10() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew11() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final double[] result = fixer.fixAndReturnNew(
                measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz,
                mzx, mzy);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew12() throws AlgebraException {
        final AccelerationFixer fixer = new AccelerationFixer();

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

        final Matrix result = fixer.fixAndReturnNewMatrix(
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
