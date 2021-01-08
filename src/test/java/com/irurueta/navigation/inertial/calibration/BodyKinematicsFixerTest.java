/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.fail;

public class BodyKinematicsFixerTest {

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
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAccelerationBias(), new Matrix(3, 1));
        final Matrix ba = new Matrix(3, 1);
        fixer.getAccelerationBias(ba);
        assertEquals(ba, new Matrix(3, 1));

        assertArrayEquals(fixer.getAccelerationBiasArray(), new double[3], 0.0);
        final double[] ba2 = new double[3];
        fixer.getAccelerationBiasArray(ba2);
        assertArrayEquals(ba2, new double[3], 0.0);
        assertEquals(fixer.getAccelerationBiasX(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationBiasY(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationBiasZ(), 0.0, 0.0);

        final AccelerationTriad accelerationTriad1 = fixer.getAccelerationBiasAsTriad();
        assertEquals(accelerationTriad1.getValueX(), 0.0, 0.0);
        assertEquals(accelerationTriad1.getValueY(), 0.0, 0.0);
        assertEquals(accelerationTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                accelerationTriad1.getUnit());
        final AccelerationTriad accelerationTriad2 = new AccelerationTriad();
        fixer.getAccelerationBiasAsTriad(accelerationTriad2);
        assertEquals(accelerationTriad1, accelerationTriad2);

        assertEquals(fixer.getAccelerationCrossCouplingErrors(),
                new Matrix(3, 3));
        final Matrix ma = new Matrix(3, 3);
        fixer.getAccelerationCrossCouplingErrors(ma);
        assertEquals(ma, new Matrix(3, 3));

        final Acceleration bax1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);
        final Acceleration bay1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);
        final Acceleration baz1 = fixer.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        assertEquals(fixer.getAccelerationSx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMxy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMxz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzy(), 0.0, 0.0);

        assertEquals(fixer.getAngularSpeedBias(), new Matrix(3, 1));
        final Matrix bg = new Matrix(3, 1);
        fixer.getAngularSpeedBias(bg);
        assertEquals(bg, new Matrix(3, 1));

        assertArrayEquals(fixer.getAngularSpeedBiasArray(), new double[3], 0.0);
        final double[] bg2 = new double[3];
        fixer.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(bg2, new double[3], 0.0);

        final AngularSpeedTriad angularSpeedTriad1 = fixer.getAngularSpeedBiasAsTriad();
        assertEquals(angularSpeedTriad1.getValueX(), 0.0, 0.0);
        assertEquals(angularSpeedTriad1.getValueY(), 0.0, 0.0);
        assertEquals(angularSpeedTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedTriad1.getUnit());
        final AngularSpeedTriad angularSpeedTriad2 = new AngularSpeedTriad();
        fixer.getAngularSpeedBiasAsTriad(angularSpeedTriad2);
        assertEquals(angularSpeedTriad1, angularSpeedTriad2);

        assertEquals(fixer.getAngularSpeedBiasX(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedBiasY(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedBiasZ(), 0.0, 0.0);

        final AngularSpeed bgx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        assertEquals(fixer.getAngularSpeedCrossCouplingErrors(),
                new Matrix(3, 3));
        final Matrix mg = new Matrix(3, 3);
        fixer.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(mg, new Matrix(3, 3));

        assertEquals(fixer.getAngularSpeedSx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMxy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMxz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzy(), 0.0, 0.0);

        assertEquals(fixer.getAngularSpeedGDependantCrossBias(),
                new Matrix(3, 3));
        final Matrix gg = new Matrix(3, 3);
        fixer.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(gg, new Matrix(3, 3));
    }

    @Test
    public void testGetSetAccelerationBias() throws WrongSizeException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final Matrix b1 = fixer.getAccelerationBias();
        final Matrix b2 = new Matrix(1, 1);
        fixer.getAccelerationBias(b2);

        assertEquals(b1, new Matrix(3, 1));
        assertEquals(b1, b2);

        // set new value
        final Matrix b3 = generateBa();
        fixer.setAccelerationBias(b3);

        // check
        final Matrix b4 = fixer.getAccelerationBias();
        final Matrix b5 = new Matrix(3, 1);
        fixer.getAccelerationBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        try {
            fixer.setAccelerationBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAccelerationBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasArray() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final double[] b1 = fixer.getAccelerationBiasArray();
        final double[] b2 = new double[3];
        fixer.getAccelerationBiasArray(b2);

        assertArrayEquals(b1, new double[3], 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final double[] b3 = generateBa().getBuffer();
        fixer.setAccelerationBias(b3);

        // check
        final double[] b4 = fixer.getAccelerationBiasArray();
        final double[] b5 = new double[3];
        fixer.getAccelerationBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        try {
            fixer.getAccelerationBiasArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAccelerationBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasTriad() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final AccelerationTriad triad1 = fixer.getAccelerationBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new value
        final AccelerationTriad triad2 = new AccelerationTriad();
        triad2.setValueCoordinates(generateBa());
        fixer.setAccelerationBias(triad2);

        // check
        final AccelerationTriad triad3 = fixer.getAccelerationBiasAsTriad();
        final AccelerationTriad triad4 = new AccelerationTriad();
        fixer.getAccelerationBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetAccelerationBiasX() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationBiasX(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        fixer.setAccelerationBiasX(bx);

        // check
        assertEquals(bx, fixer.getAccelerationBiasX(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasY() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationBiasY(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double by = b.getElementAtIndex(1);
        fixer.setAccelerationBiasY(by);

        // check
        assertEquals(by, fixer.getAccelerationBiasY(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasZ() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBa();
        final double bz = b.getElementAtIndex(2);
        fixer.setAccelerationBiasZ(bz);

        // check
        assertEquals(bz, fixer.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testSetAccelerationBias1() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAccelerationBiasX(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationBiasY(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        final double by = b.getElementAtIndex(1);
        final double bz = b.getElementAtIndex(2);
        fixer.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getAccelerationBiasX(), 0.0);
        assertEquals(by, fixer.getAccelerationBiasY(), 0.0);
        assertEquals(bz, fixer.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasXAsAcceleration() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final Acceleration bx1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double bx = b.getElementAtIndex(0);
        final Acceleration bx2 = new Acceleration(
                bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasX(bx2);

        // check
        final Acceleration bx3 = fixer.getAccelerationBiasXAsAcceleration();
        final Acceleration bx4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasXAsAcceleration(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetAccelerationBiasYAsAcceleration() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final Acceleration by1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double by = b.getElementAtIndex(1);
        final Acceleration by2 = new Acceleration(
                by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasY(by2);

        // check
        final Acceleration by3 = fixer.getAccelerationBiasYAsAcceleration();
        final Acceleration by4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasYAsAcceleration(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetAccelerationBiasZAsAcceleration() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final Acceleration bz1 = fixer.getAccelerationBiasZAsAcceleration();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new value
        final Matrix b = generateBa();
        final double bz = b.getElementAtIndex(2);
        final Acceleration bz2 = new Acceleration(
                bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasZ(bz2);

        // check
        final Acceleration bz3 = fixer.getAccelerationBiasZAsAcceleration();
        final Acceleration bz4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasZAsAcceleration(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testGetSetBias2() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        final Acceleration bx1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        final Acceleration by1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        final Acceleration bz1 = fixer.getAccelerationBiasZAsAcceleration();
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
        fixer.setAccelerationBias(bx2, by2, bz2);

        // check
        final Acceleration bx3 = fixer.getAccelerationBiasXAsAcceleration();
        final Acceleration by3 = fixer.getAccelerationBiasYAsAcceleration();
        final Acceleration bz3 = fixer.getAccelerationBiasZAsAcceleration();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testGetSetAccelerationCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        final Matrix m1 = fixer.getAccelerationCrossCouplingErrors();
        final Matrix m2 = new Matrix(1, 1);
        fixer.getAccelerationCrossCouplingErrors(m2);

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

        fixer.setAccelerationCrossCouplingErrors(m3);

        // check
        final Matrix m4 = fixer.getAccelerationCrossCouplingErrors();
        final Matrix m5 = new Matrix(3, 3);
        fixer.getAccelerationCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);

        // Force IllegalArgumentException
        try {
            fixer.setAccelerationCrossCouplingErrors(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAccelerationCrossCouplingErrors(
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force AlgebraException
        try {
            final Matrix wrong = Matrix.identity(3, 3);
            wrong.multiplyByScalar(-1.0);
            fixer.setAccelerationCrossCouplingErrors(wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationSx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationSx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sx = m.getElementAt(0, 0);

        fixer.setAccelerationSx(sx);

        // check
        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationSy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationSy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sy = m.getElementAt(1, 1);

        fixer.setAccelerationSy(sy);

        // check
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
    }

    @Test
    public void testGetSetAccelerationSz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationSz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double sz = m.getElementAt(2, 2);

        fixer.setAccelerationSz(sz);

        // check
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMxy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMxy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mxy = m.getElementAt(0, 1);

        fixer.setAccelerationMxy(mxy);

        // check
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMxz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMxz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mxz = m.getElementAt(0, 2);

        fixer.setAccelerationMxz(mxz);

        // check
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMyx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMyx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double myx = m.getElementAt(1, 0);

        fixer.setAccelerationMyx(myx);

        // check
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMyz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMyz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double myz = m.getElementAt(1, 2);

        fixer.setAccelerationMyz(myz);

        // check
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMzx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMzx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mzx = m.getElementAt(2, 0);

        fixer.setAccelerationMzx(mzx);

        // check
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMzy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAccelerationMzy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMa();
        final double mzy = m.getElementAt(2, 1);

        fixer.setAccelerationMzy(mzy);

        // check
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testSetAccelerationScalingFactors() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAccelerationSx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSz(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMa();
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);

        fixer.setAccelerationScalingFactors(sx, sy, sz);

        // check
        assertEquals(fixer.getAccelerationSx(), sx, 0.0);
        assertEquals(fixer.getAccelerationSy(), sy, 0.0);
        assertEquals(fixer.getAccelerationSz(), sz, 0.0);
    }

    @Test
    public void testSetAccelerationCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAccelerationMxy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMxz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMa();
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setAccelerationCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testSetAccelerationScalingFactorsAndCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAccelerationSx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationSz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMxy(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMxz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMyz(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzx(), 0.0, 0.0);
        assertEquals(fixer.getAccelerationMzy(), 0.0, 0.0);

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

        fixer.setAccelerationScalingFactorsAndCrossCouplingErrors(sx, sy, sz,
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(fixer.getAccelerationSx(), sx, 0.0);
        assertEquals(fixer.getAccelerationSy(), sy, 0.0);
        assertEquals(fixer.getAccelerationSz(), sz, 0.0);
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBias() throws WrongSizeException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final Matrix b1 = fixer.getAngularSpeedBias();
        final Matrix b2 = new Matrix(1, 1);
        fixer.getAngularSpeedBias(b2);

        assertEquals(b1, new Matrix(3, 1));
        assertEquals(b1, b2);

        // set new value
        final Matrix b3 = generateBg();
        fixer.setAngularSpeedBias(b3);

        // check
        final Matrix b4 = fixer.getAngularSpeedBias();
        final Matrix b5 = new Matrix(3, 1);
        fixer.getAngularSpeedBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        try {
            fixer.setAngularSpeedBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAngularSpeedBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedBiasArray() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final double[] b1 = fixer.getAngularSpeedBiasArray();
        final double[] b2 = new double[3];
        fixer.getAngularSpeedBiasArray(b2);

        assertArrayEquals(b1, new double[3], 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final double[] b3 = generateBg().getBuffer();
        fixer.setAngularSpeedBias(b3);

        // check
        final double[] b4 = fixer.getAngularSpeedBiasArray();
        final double[] b5 = new double[3];
        fixer.getAngularSpeedBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        try {
            fixer.getAngularSpeedBiasArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAngularSpeedBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedBiasTriad() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final AngularSpeedTriad triad1 = fixer.getAngularSpeedBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        triad2.setValueCoordinates(generateBg());
        fixer.setAngularSpeedBias(triad2);

        // check
        final AngularSpeedTriad triad3 = fixer.getAngularSpeedBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        fixer.getAngularSpeedBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetAngularSpeedBiasX() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedBiasX(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        fixer.setAngularSpeedBiasX(bx);

        // check
        assertEquals(bx, fixer.getAngularSpeedBiasX(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasY() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedBiasY(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double by = b.getElementAtIndex(1);
        fixer.setAngularSpeedBiasY(by);

        // check
        assertEquals(by, fixer.getAngularSpeedBiasY(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasZ() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix b = generateBg();
        final double bz = b.getElementAtIndex(2);
        fixer.setAngularSpeedBiasZ(bz);

        // check
        assertEquals(bz, fixer.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    public void testSetAngularSpeedBias1() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAngularSpeedBiasX(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedBiasY(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        final double by = b.getElementAtIndex(1);
        final double bz = b.getElementAtIndex(2);
        fixer.setAngularSpeedBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getAngularSpeedBiasX(), 0.0);
        assertEquals(by, fixer.getAngularSpeedBiasY(), 0.0);
        assertEquals(bz, fixer.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasXAsAngularSpeed() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final AngularSpeed bx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        // set new value
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        final AngularSpeed bx2 = new AngularSpeed(
                bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasX(bx2);

        // check
        final AngularSpeed bx3 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        final AngularSpeed bx4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasXAsAngularSpeed(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetAngularSpeedBiasYAsAngularSpeed() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final AngularSpeed by1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        // set new value
        final Matrix b = generateBg();
        final double by = b.getElementAtIndex(1);
        final AngularSpeed by2 = new AngularSpeed(
                by, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasY(by2);

        // check
        final AngularSpeed by3 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        final AngularSpeed by4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasYAsAngularSpeed(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetAngularSpeedBiasZAsAngularSpeed() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        final AngularSpeed bz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new value
        final Matrix b = generateBg();
        final double bz = b.getElementAtIndex(2);
        final AngularSpeed bz2 = new AngularSpeed(
                bz, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasZ(bz2);

        // check
        final AngularSpeed bz3 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        final AngularSpeed bz4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasZAsAngularSpeed(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testGetSetAngularSpeedBias2() {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        final AngularSpeed bx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        final AngularSpeed by1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        final AngularSpeed bz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new values
        final Matrix b = generateBg();
        final double bx = b.getElementAtIndex(0);
        final double by = b.getElementAtIndex(1);
        final double bz = b.getElementAtIndex(2);
        final AngularSpeed bx2 = new AngularSpeed(
                bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(
                by, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bz2 = new AngularSpeed(
                bz, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBias(bx2, by2, bz2);

        // check
        final AngularSpeed bx3 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        final AngularSpeed by3 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        final AngularSpeed bz3 = fixer.getAngularSpeedBiasZAsAngularSpeed();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testGetSetAngularSpeedCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        final Matrix m1 = fixer.getAngularSpeedCrossCouplingErrors();
        final Matrix m2 = new Matrix(1, 1);
        fixer.getAngularSpeedCrossCouplingErrors(m2);

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

        fixer.setAngularSpeedCrossCouplingErrors(m3);

        // check
        final Matrix m4 = fixer.getAngularSpeedCrossCouplingErrors();
        final Matrix m5 = new Matrix(3, 3);
        fixer.getAngularSpeedCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);

        // Force IllegalArgumentException
        try {
            fixer.setAngularSpeedCrossCouplingErrors(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAngularSpeedCrossCouplingErrors(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force AlgebraException
        try {
            final Matrix wrong = Matrix.identity(3, 3);
            wrong.multiplyByScalar(-1.0);
            fixer.setAngularSpeedCrossCouplingErrors(wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedSx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedSx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sx = m.getElementAt(0, 0);

        fixer.setAngularSpeedSx(sx);

        // check
        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedSy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedSy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sy = m.getElementAt(1, 1);

        fixer.setAngularSpeedSy(sy);

        // check
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedSz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedSz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double sz = m.getElementAt(2, 2);

        fixer.setAngularSpeedSz(sz);

        // check
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMxy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMxy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mxy = m.getElementAt(0, 1);

        fixer.setAngularSpeedMxy(mxy);

        // check
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMxz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMxz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mxz = m.getElementAt(0, 2);

        fixer.setAngularSpeedMxz(mxz);

        // check
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMyx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMyx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double myx = m.getElementAt(1, 0);

        fixer.setAngularSpeedMyx(myx);

        // check
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMyz() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMyz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double myz = m.getElementAt(1, 2);

        fixer.setAngularSpeedMyz(myz);

        // check
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMzx() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMzx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mzx = m.getElementAt(2, 0);

        fixer.setAngularSpeedMzx(mzx);

        // check
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMzy() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(fixer.getAngularSpeedMzy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateMg();
        final double mzy = m.getElementAt(2, 1);

        fixer.setAngularSpeedMzy(mzy);

        // check
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testSetAngularSpeedScalingFactors() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAngularSpeedSx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSz(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMg();
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);

        fixer.setAngularSpeedScalingFactors(sx, sy, sz);

        // check
        assertEquals(fixer.getAngularSpeedSx(), sx, 0.0);
        assertEquals(fixer.getAngularSpeedSy(), sy, 0.0);
        assertEquals(fixer.getAngularSpeedSz(), sz, 0.0);
    }

    @Test
    public void testSetAngularSpeedCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAngularSpeedMxy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMxz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateMg();
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setAngularSpeedCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testSetAngularSpeedScalingFactorsAndCrossCouplingErrors()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(fixer.getAngularSpeedSx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedSz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMxy(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMxz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMyz(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzx(), 0.0, 0.0);
        assertEquals(fixer.getAngularSpeedMzy(), 0.0, 0.0);

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

        fixer.setAngularSpeedScalingFactorsAndCrossCouplingErrors(sx, sy, sz,
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(fixer.getAngularSpeedSx(), sx, 0.0);
        assertEquals(fixer.getAngularSpeedSy(), sy, 0.0);
        assertEquals(fixer.getAngularSpeedSz(), sz, 0.0);
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedGDependantCrossBias()
            throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        // check default values
        final Matrix g1 = fixer.getAngularSpeedGDependantCrossBias();
        final Matrix g2 = new Matrix(1, 1);
        fixer.getAngularSpeedGDependantCrossBias(g2);

        assertEquals(g1, new Matrix(3, 3));
        assertEquals(g1, g2);

        // set new values
        final Matrix g3 = generateGg();

        fixer.setAngularSpeedGDependantCrossBias(g3);

        // check
        final Matrix g4 = fixer.getAngularSpeedGDependantCrossBias();
        final Matrix g5 = new Matrix(3, 3);
        fixer.getAngularSpeedGDependantCrossBias(g5);

        assertEquals(g3, g4);
        assertEquals(g3, g5);

        // Force IllegalArgumentException
        try {
            fixer.setAngularSpeedGDependantCrossBias(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.setAngularSpeedGDependantCrossBias(
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix1() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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

        final BodyKinematics result = new BodyKinematics();
        fixer.fix(measuredKinematics, result);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    public void testFix2() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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
        final AccelerationTriad trueF = trueKinematics.getSpecificForceTriad();
        final AngularSpeedTriad trueAngularSpeed = trueKinematics
                .getAngularRateTriad();

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());
        final AccelerationTriad measuredF = measuredKinematics
                .getSpecificForceTriad();
        final AngularSpeedTriad measuredAngularSpeed = measuredKinematics
                .getAngularRateTriad();

        final AccelerationTriad fixedF = new AccelerationTriad();
        final AngularSpeedTriad fixedAngularSpeed = new AngularSpeedTriad();
        fixer.fix(measuredF, measuredAngularSpeed, fixedF, fixedAngularSpeed);

        // check
        assertTrue(fixedF.equals(trueF, ABSOLUTE_ERROR));
        assertTrue(fixedAngularSpeed.equals(trueAngularSpeed, ABSOLUTE_ERROR));
    }

    @Test
    public void testFix3() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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
        final AccelerationTriad trueF = trueKinematics.getSpecificForceTriad();
        final AngularSpeedTriad trueAngularSpeed = trueKinematics
                .getAngularRateTriad();

        final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                        new Random());

        final AccelerationTriad fixedF = new AccelerationTriad();
        final AngularSpeedTriad fixedAngularSpeed = new AngularSpeedTriad();
        fixer.fix(measuredKinematics, fixedF, fixedAngularSpeed);

        // check
        assertTrue(fixedF.equals(trueF, ABSOLUTE_ERROR));
        assertTrue(fixedAngularSpeed.equals(trueAngularSpeed, ABSOLUTE_ERROR));
    }

    @Test
    public void testFix4() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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
        final AngularSpeedTriad measuredAngularSpeed = measuredKinematics
                .getAngularRateTriad();

        final BodyKinematics result = new BodyKinematics();
        fixer.fix(measuredF, measuredAngularSpeed, result);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    public void fixAndReturnNew1() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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

        final BodyKinematics result = fixer.fixAndReturnNew(measuredKinematics);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    public void fixAndReturnNew2() throws AlgebraException {
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

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
        final AngularSpeedTriad measuredAngularSpeed = measuredKinematics
                .getAngularRateTriad();

        final BodyKinematics result = fixer.fixAndReturnNew(
                measuredF, measuredAngularSpeed);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
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
