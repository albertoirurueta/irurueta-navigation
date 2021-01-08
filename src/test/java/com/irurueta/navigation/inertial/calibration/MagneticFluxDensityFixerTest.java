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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

import static org.junit.Assert.*;

public class MagneticFluxDensityFixerTest {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    public void testConstructor() throws WrongSizeException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

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

        final BodyMagneticFluxDensity bb1 = fixer.getBiasAsBodyMagneticFluxDensity();
        assertEquals(bb1.getBx(), 0.0, 0.0);
        assertEquals(bb1.getBy(), 0.0, 0.0);
        assertEquals(bb1.getBz(), 0.0, 0.0);
        final BodyMagneticFluxDensity bb2 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(bb2);
        assertEquals(bb1, bb2);

        final MagneticFluxDensityTriad triad1 = fixer.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        fixer.getBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(fixer.getCrossCouplingErrors(),
                new Matrix(3, 3));
        final Matrix m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(m, new Matrix(3, 3));

        final MagneticFluxDensity bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasXAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        final MagneticFluxDensity by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasYAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        final MagneticFluxDensity bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasZAsMagneticFluxDensity(bz2);
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
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final Matrix b1 = fixer.getBias();
        final Matrix b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(b1, new Matrix(3, 1));
        assertEquals(b1, b2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b3 = Matrix.newFromArray(generateHardIron(randomizer));
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
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final double[] b1 = fixer.getBiasArray();
        final double[] b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(b1, new double[3], 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b3 = generateHardIron(randomizer);
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
    public void testGetSetBiasAsBodyMagneticFluxDensity() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final BodyMagneticFluxDensity b1 = fixer.getBiasAsBodyMagneticFluxDensity();
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(b2);

        assertEquals(b1.getBx(), 0.0, 0.0);
        assertEquals(b1.getBy(), 0.0, 0.0);
        assertEquals(b1.getBz(), 0.0, 0.0);
        assertEquals(b1, b2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final BodyMagneticFluxDensity b3 = new BodyMagneticFluxDensity(
                b[0], b[1], b[2]);
        fixer.setBias(b3);

        // check
        final BodyMagneticFluxDensity b4 = fixer.getBiasAsBodyMagneticFluxDensity();
        final BodyMagneticFluxDensity b5 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);
    }

    @Test
    public void testGetSetBiasTriad() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final MagneticFluxDensityTriad triad1 = fixer.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());

        // set new value
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        triad2.setValueCoordinates(generateHardIron(randomizer));
        fixer.setBias(triad2);

        // check
        final MagneticFluxDensityTriad triad3 = fixer.getBiasAsTriad();
        final MagneticFluxDensityTriad triad4 = new MagneticFluxDensityTriad();
        fixer.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetBiasX() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getBiasX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bx = b[0];
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    public void testGetSetBiasY() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getBiasY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double by = b[1];
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    public void testGetSetBiasZ() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bz = b[2];
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBias1() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getBiasX(), 0.0, 0.0);
        assertEquals(fixer.getBiasY(), 0.0, 0.0);
        assertEquals(fixer.getBiasZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bx = b[0];
        final double by = b[1];
        final double bz = b[2];
        fixer.setBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
        assertEquals(by, fixer.getBiasY(), 0.0);
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetBiasXAsMagneticFluxDensity() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final MagneticFluxDensity bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bx = b[0];
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                bx, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasX(bx2);

        // check
        final MagneticFluxDensity bx3 = fixer.getBiasXAsMagneticFluxDensity();
        final MagneticFluxDensity bx4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasXAsMagneticFluxDensity(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetBiasYAsMagneticFluxDensity() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final MagneticFluxDensity by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double by = b[1];
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                by, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasY(by2);

        // check
        final MagneticFluxDensity by3 = fixer.getBiasYAsMagneticFluxDensity();
        final MagneticFluxDensity by4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasYAsMagneticFluxDensity(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetBiasZAsMagneticFluxDensity() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        final MagneticFluxDensity bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bz = b[2];
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                bz, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasZ(bz2);

        // check
        final MagneticFluxDensity bz3 = fixer.getBiasZAsMagneticFluxDensity();
        final MagneticFluxDensity bz4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasZAsMagneticFluxDensity(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetBias2() {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default values
        final MagneticFluxDensity bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        final MagneticFluxDensity by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        final MagneticFluxDensity bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double bx = b[0];
        final double by = b[1];
        final double bz = b[2];
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                bx, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                by, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                bz, MagneticFluxDensityUnit.TESLA);
        fixer.setBias(bx2, by2, bz2);

        // check
        final MagneticFluxDensity bx3 = fixer.getBiasXAsMagneticFluxDensity();
        final MagneticFluxDensity by3 = fixer.getBiasYAsMagneticFluxDensity();
        final MagneticFluxDensity bz3 = fixer.getBiasZAsMagneticFluxDensity();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testGetSetCrossCouplingErrors() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default values
        final Matrix m1 = fixer.getCrossCouplingErrors();
        final Matrix m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(m1, new Matrix(3, 3));
        assertEquals(m1, m2);

        // set new values
        final Matrix m3 = generateSoftIronGeneral();
        assertNotNull(m3);
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
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getSx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    public void testGetSetSy() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getSy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    public void testGetSetSz() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    public void testGetSetMxy() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMxy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    public void testGetSetMxz() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMxz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    public void testGetSetMyx() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMyx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    public void testGetSetMyz() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMyz(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    public void testGetSetMzx() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMzx(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    public void testGetSetMzy() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new value
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testSetScalingFactors() throws AlgebraException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(fixer.getSx(), 0.0, 0.0);
        assertEquals(fixer.getSy(), 0.0, 0.0);
        assertEquals(fixer.getSz(), 0.0, 0.0);

        // set new values
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
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
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(fixer.getMxy(), 0.0, 0.0);
        assertEquals(fixer.getMxz(), 0.0, 0.0);
        assertEquals(fixer.getMyx(), 0.0, 0.0);
        assertEquals(fixer.getMyz(), 0.0, 0.0);
        assertEquals(fixer.getMzx(), 0.0, 0.0);
        assertEquals(fixer.getMzy(), 0.0, 0.0);

        // set new values
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
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
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

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
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
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
    public void testFix1() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredB, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix2() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final MagneticFluxDensityTriad measuredB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m).getCoordinatesAsTriad();

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredB, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix3() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final Matrix result = new Matrix(
                BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredB, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix4() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final MagneticFluxDensityTriad measuredB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m).getCoordinatesAsTriad();

        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measuredB, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix5() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        fixer.fix(measuredB, result);

        // check
        assertTrue(result.equals(trueB, ABSOLUTE_ERROR));
    }

    @Test
    public void testFix6() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final MagneticFluxDensityTriad measuredB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m).getCoordinatesAsTriad();

        final MagneticFluxDensityTriad result = new MagneticFluxDensityTriad();
        fixer.fix(measuredB, result);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFix7() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final MagneticFluxDensity measuredBx = measuredB.getBxAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBy = measuredB.getByAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBz = measuredB.getBzAsMagneticFluxDensity();

        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        fixer.fix(measuredBx, measuredBy, measuredBz, result);

        // check
        assertTrue(result.equals(trueB, ABSOLUTE_ERROR));
    }

    @Test
    public void testFix8() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final MagneticFluxDensity measuredBx = measuredB.getBxAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBy = measuredB.getByAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBz = measuredB.getBzAsMagneticFluxDensity();

        final MagneticFluxDensityTriad result = new MagneticFluxDensityTriad();
        fixer.fix(measuredBx, measuredBy, measuredBz, result);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFix9() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double[] measuredB = measB.asArray();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix10() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final Matrix measuredB = measB.asMatrix();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix11() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final Matrix measuredB = measB.asMatrix();

        final Matrix result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix12() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measuredB.getBx();
        final double measBy = measuredB.getBy();
        final double measBz = measuredB.getBz();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix13() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measuredB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measuredB.getBx();
        final double measBy = measuredB.getBy();
        final double measBz = measuredB.getBz();

        final Matrix result = new Matrix(
                BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix14() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final double[] measuredB = measB.asArray();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new double[1], b, m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(1, 1), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, m, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix15() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final Matrix measuredB = measB.asMatrix();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), b, m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), b, m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(1, 1), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, m, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix16() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final Matrix measuredB = measB.asMatrix();

        final Matrix result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(new Matrix(1, 1), b, m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(new Matrix(3, 3), b, m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(1, 1), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, new Matrix(3, 3), m, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, new Matrix(1, 3), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, m, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measuredB, b, m, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix17() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measB.getBx();
        final double measBy = measB.getBy();
        final double measBz = measB.getBz();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix18() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measB.getBx();
        final double measBy = measB.getBy();
        final double measBz = measB.getBz();

        final Matrix result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    new Matrix(1, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    new Matrix(3, 1), result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix19() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);


        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measB.getBx();
        final double measBy = measB.getBy();
        final double measBz = measB.getBz();

        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);

        // check
        assertEquals(result[0], trueBx, ABSOLUTE_ERROR);
        assertEquals(result[1], trueBy, ABSOLUTE_ERROR);
        assertEquals(result[2], trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFix20() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);


        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final double trueBx = trueB.getBx();
        final double trueBy = trueB.getBy();
        final double trueBz = trueB.getBz();

        final BodyMagneticFluxDensity measB =
                BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final double measBx = measB.getBx();
        final double measBy = measB.getBy();
        final double measBz = measB.getBz();

        final Matrix result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);

        // check
        assertEquals(result.getElementAtIndex(0), trueBx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), trueBy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), trueBz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                    sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew1() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final MagneticFluxDensityTriad measuredB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m).getCoordinatesAsTriad();

        final MagneticFluxDensityTriad result = fixer.fixAndReturnNew(measuredB);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFixAndReturnNew2() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measuredB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final MagneticFluxDensity measuredBx = measuredB
                .getBxAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBy = measuredB
                .getByAsMagneticFluxDensity();
        final MagneticFluxDensity measuredBz = measuredB
                .getBzAsMagneticFluxDensity();

        final MagneticFluxDensityTriad result = fixer.fixAndReturnNew(
                measuredBx, measuredBy, measuredBz);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFixAndReturnNew3() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double[] measuredB = measB.asArray();

        final double[] result = fixer.fixAndReturnNew(measuredB);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new double[1]);
            fail("IllegalArgumentException expected but ot thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew4() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final Matrix measuredB = measB.asMatrix();

        final double[] result = fixer.fixAndReturnNew(measuredB);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNew(new Matrix(1, 1));
            fail("IllegalArgumentException expected but ot thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNew(new Matrix(3, 3));
            fail("IllegalArgumentException expected but ot thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNewMatrix1() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final Matrix measuredB = measB.asMatrix();

        final Matrix result = fixer.fixAndReturnNewMatrix(measuredB);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            fixer.fixAndReturnNewMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but ot thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fixer.fixAndReturnNewMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but ot thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFixAndReturnNew5() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final double[] result = fixer.fixAndReturnNew(
                measuredBx, measuredBy, measuredBz);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix2() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredBx, measuredBy, measuredBz);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFixAndReturnNew6() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b.getBuffer(), m);
        final double[] measuredB = measB.asArray();

        final double[] result = fixer.fixAndReturnNew(measuredB, b, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew7() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b.getBuffer(), m);
        final Matrix measuredB = measB.asMatrix();

        final double[] result = fixer.fixAndReturnNew(measuredB, b, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix3() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix b = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b.getBuffer(), m);
        final Matrix measuredB = measB.asMatrix();

        final Matrix result = fixer.fixAndReturnNewMatrix(measuredB, b, m);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFixAndReturnNew8() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final double[] result = fixer.fixAndReturnNew(
                measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix4() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ, m);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testFixAndReturnNew9() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final double[] result = fixer.fixAndReturnNew(
                measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix5() throws AlgebraException, IOException {
        final MagneticFluxDensityFixer fixer = new MagneticFluxDensityFixer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] b = generateHardIron(randomizer);
        final double biasX = b[0];
        final double biasY = b[1];
        final double biasZ = b[2];
        final Matrix m = generateSoftIronGeneral();
        assertNotNull(m);
        final double sx = m.getElementAt(0, 0);
        final double sy = m.getElementAt(1, 1);
        final double sz = m.getElementAt(2, 2);
        final double mxy = m.getElementAt(0, 1);
        final double mxz = m.getElementAt(0, 2);
        final double myx = m.getElementAt(1, 0);
        final double myz = m.getElementAt(1, 2);
        final double mzx = m.getElementAt(2, 0);
        final double mzy = m.getElementAt(2, 1);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity trueB =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensity measB = BodyMagneticFluxDensityGenerator
                .generate(trueB, b, m);
        final double measuredBx = measB.getBx();
        final double measuredBy = measB.getBy();
        final double measuredBz = measB.getBz();

        final Matrix result = fixer.fixAndReturnNewMatrix(
                measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }


    private static CoordinateTransformation generateBodyC(
            final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
    }

    private static double[] generateHardIron(
            final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(
            final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
