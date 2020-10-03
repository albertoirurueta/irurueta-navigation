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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class BodyMagneticFluxDensityTest {

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.

    private static final double MIN_VALUE = 30e-6;
    private static final double MAX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(b.getBx(), 0.0, 0.0);
        assertEquals(b.getBy(), 0.0, 0.0);
        assertEquals(b.getBz(), 0.0, 0.0);
        assertEquals(b.getNorm(), 0.0, 0.0);
        MagneticFluxDensity bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity bx2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        MagneticFluxDensity by2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        MagneticFluxDensity bz2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        MagneticFluxDensityTriad triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        b = new BodyMagneticFluxDensity(bx, by, bz);

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bx * bx + by * by + bz * bz),
                0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), bx, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), by, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), bz, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bx, 0.0);
        assertEquals(triad1.getValueY(), by, 0.0);
        assertEquals(triad1.getValueZ(), bz, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density measurements
        bx1 = new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
        by1 = new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
        bz1 = new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
        b = new BodyMagneticFluxDensity(bx1, by1, bz1);

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bx * bx + by * by + bz * bz),
                0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), bx, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), by, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), bz, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bx, 0.0);
        assertEquals(triad1.getValueY(), by, 0.0);
        assertEquals(triad1.getValueZ(), bz, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density triad
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA, bx, by, bz);
        b = new BodyMagneticFluxDensity(triad);

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bx * bx + by * by + bz * bz),
                0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), bx, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), by, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), bz, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bx, 0.0);
        assertEquals(triad1.getValueY(), by, 0.0);
        assertEquals(triad1.getValueZ(), bz, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test copy constructor
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity(b);

        assertEquals(b2.getBx(), bx, 0.0);
        assertEquals(b2.getBy(), by, 0.0);
        assertEquals(b2.getBz(), bz, 0.0);
        assertEquals(b2.getNorm(), b.getNorm(), 0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), bx, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), by, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), bz, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bx, 0.0);
        assertEquals(triad1.getValueY(), by, 0.0);
        assertEquals(triad1.getValueZ(), bz, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);
    }

    @Test
    public void testGetSetBx() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(b.getBx(), 0.0, 0.0);

        // set value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBx(bx);

        // check
        assertEquals(b.getBx(), bx, 0.0);
    }

    @Test
    public void testGetSetBxAsMagneticFluxDensity() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                bx, MagneticFluxDensityUnit.TESLA);
        b.setBx(bx2);

        // check
        final MagneticFluxDensity bx3 = b.getBxAsMagneticFluxDensity();
        final MagneticFluxDensity bx4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetBy() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(b.getBy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBy(by);

        // check
        assertEquals(b.getBy(), by, 0.0);
    }

    @Test
    public void testGetSetByAsMagneticFluxDensity() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                by, MagneticFluxDensityUnit.TESLA);
        b.setBy(by2);

        // check
        final MagneticFluxDensity by3 = b.getByAsMagneticFluxDensity();
        final MagneticFluxDensity by4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetBz() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(b.getBz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBz(bz);

        // check
        assertEquals(b.getBz(), bz, 0.0);
    }

    @Test
    public void testGetSetBzAsMagneticFluxDensity() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                bz, MagneticFluxDensityUnit.TESLA);
        b.setBz(bz2);

        // check
        final MagneticFluxDensity bz3 = b.getBzAsMagneticFluxDensity();
        final MagneticFluxDensity bz4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetCoordinates1() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(b.getBx(), 0.0, 0.0);
        assertEquals(b.getBy(), 0.0, 0.0);
        assertEquals(b.getBz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
    }

    @Test
    public void testSetCoordinates2() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(b.getBx(), 0.0, 0.0);
        assertEquals(b.getBy(), 0.0, 0.0);
        assertEquals(b.getBz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA));

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
    }

    @Test
    public void testSetCoordinates3() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(b.getBx(), 0.0, 0.0);
        assertEquals(b.getBy(), 0.0, 0.0);
        assertEquals(b.getBz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA, bx, by, bz);
        b.setCoordinates(triad1);

        // check default values
        assertEquals(b.getBx(), bx, 0.0);
        assertEquals(b.getBy(), by, 0.0);
        assertEquals(b.getBz(), bz, 0.0);
        final MagneticFluxDensityTriad triad2 = b.getCoordinatesAsTriad();
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad3);
        assertEquals(triad1, triad2);
        assertEquals(triad1, triad3);
    }

    @Test
    public void testGetNorm() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(b.getNorm(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check
        assertEquals(b.getNorm(), Math.sqrt(bx * bx + by * by + bz * bz),
                0.0);
    }

    @Test
    public void testGetNormAsMagneticFluxDensity() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity norm1 = b.getNormAsMagneticFluxDensity();
        assertEquals(norm1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(norm1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check
        final double norm = Math.sqrt(bx * bx + by * by + bz * bz);
        final MagneticFluxDensity norm2 = b.getNormAsMagneticFluxDensity();
        assertEquals(norm2.getValue().doubleValue(), norm, 0.0);
        assertEquals(norm2.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity norm3 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getNormAsMagneticFluxDensity(norm3);
        assertEquals(norm2, norm3);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity();

        b1.copyTo(b2);

        // check
        assertEquals(b1.getBx(), b2.getBx(), 0.0);
        assertEquals(b1.getBy(), b2.getBy(), 0.0);
        assertEquals(b1.getBz(), b2.getBz(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity();

        b2.copyFrom(b1);

        // check
        assertEquals(b1.getBx(), b2.getBx(), 0.0);
        assertEquals(b1.getBy(), b2.getBy(), 0.0);
        assertEquals(b1.getBz(), b2.getBz(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity(
                bx, by, bz);

        final double[] array1 = b.asArray();
        final double[] array2 = new double[
                BodyMagneticFluxDensity.COMPONENTS];
        b.asArray(array2);

        // check
        assertEquals(array1[0], bx, 0.0);
        assertEquals(array1[1], by, 0.0);
        assertEquals(array1[2], bz, 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        try {
            b.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity(
                bx, by, bz);

        final Matrix m1 = b.asMatrix();
        final Matrix m2 = new Matrix(1, 1);
        b.asMatrix(m2);
        final Matrix m3 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                1);
        b.asMatrix(m3);

        // check
        assertEquals(m1.getElementAtIndex(0), bx, 0.0);
        assertEquals(m1.getElementAtIndex(1), by, 0.0);
        assertEquals(m1.getElementAtIndex(2), bz, 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b3 = new BodyMagneticFluxDensity();

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b3 = new BodyMagneticFluxDensity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(b1.equals((Object) b1));
        assertTrue(b1.equals(b1));
        assertTrue(b1.equals(b2));
        assertFalse(b1.equals(b3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(b1.equals((Object) null));
        assertFalse(b1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(b1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b2 = new BodyMagneticFluxDensity(
                bx, by, bz);
        final BodyMagneticFluxDensity b3 = new BodyMagneticFluxDensity();

        assertTrue(b1.equals(b1, THRESHOLD));
        assertTrue(b1.equals(b2, THRESHOLD));
        assertFalse(b1.equals(b3, THRESHOLD));
        assertFalse(b1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final BodyMagneticFluxDensity b1 = new BodyMagneticFluxDensity(
                bx, by, bz);

        final Object b2 = b1.clone();

        // check
        assertEquals(b1, b2);
    }
}
