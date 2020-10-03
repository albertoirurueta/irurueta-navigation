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

public class NEDMagneticFluxDensityTest {

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.

    private static final double MIN_VALUE = 30e-6;
    private static final double MAX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);
        assertEquals(b.getNorm(), 0.0, 0.0);
        MagneticFluxDensity bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity bn2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        MagneticFluxDensity be2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        MagneticFluxDensity bd2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
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
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        b = new NEDMagneticFluxDensity(bn, be, bd);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), bn, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), be, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), bd, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bn2 = new MagneticFluxDensity(bn, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(be, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(bd, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bn, 0.0);
        assertEquals(triad1.getValueY(), be, 0.0);
        assertEquals(triad1.getValueZ(), bd, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density measurements
        bn1 = new MagneticFluxDensity(bn, MagneticFluxDensityUnit.TESLA);
        be1 = new MagneticFluxDensity(be, MagneticFluxDensityUnit.TESLA);
        bd1 = new MagneticFluxDensity(bd, MagneticFluxDensityUnit.TESLA);
        b = new NEDMagneticFluxDensity(bn1, be1, bd1);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), bn, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), be, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), bd, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bn2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bn, 0.0);
        assertEquals(triad1.getValueY(), be, 0.0);
        assertEquals(triad1.getValueZ(), bd, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density triad
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA, bn, be, bd);
        b = new NEDMagneticFluxDensity(triad);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), bn, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), be, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), bd, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bn2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bn, 0.0);
        assertEquals(triad1.getValueY(), be, 0.0);
        assertEquals(triad1.getValueZ(), bd, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test copy constructor
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(b);

        assertEquals(b2.getBn(), bn, 0.0);
        assertEquals(b2.getBe(), be, 0.0);
        assertEquals(b2.getBd(), bd, 0.0);
        assertEquals(b2.getNorm(), b.getNorm(),
                0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), bn, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), be, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), bd, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);
        bn2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(triad1.getValueX(), bn, 0.0);
        assertEquals(triad1.getValueY(), be, 0.0);
        assertEquals(triad1.getValueZ(), bd, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);
    }

    @Test
    public void testGetSetBn() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBn(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBn(bn);

        // check
        assertEquals(b.getBn(), bn, 0.0);
    }

    @Test
    public void testGetSetBnAsMagneticFluxDensity() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bn1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity bn2 = new MagneticFluxDensity(
                bn, MagneticFluxDensityUnit.TESLA);
        b.setBn(bn2);

        // check
        final MagneticFluxDensity bn3 = b.getBnAsMagneticFluxDensity();
        final MagneticFluxDensity bn4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn4);

        assertEquals(bn2, bn3);
        assertEquals(bn2, bn4);
    }

    @Test
    public void testGetSetBe() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBe(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBe(be);

        // check
        assertEquals(b.getBe(), be, 0.0);
    }

    @Test
    public void testGetSetBeAsMagneticFluxDensity() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(be1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity be2 = new MagneticFluxDensity(
                be, MagneticFluxDensityUnit.TESLA);
        b.setBe(be2);

        // check
        final MagneticFluxDensity be3 = b.getBeAsMagneticFluxDensity();
        final MagneticFluxDensity be4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be4);

        assertEquals(be2, be3);
        assertEquals(be2, be4);
    }

    @Test
    public void testGetSetBd() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBd(bd);

        // check
        assertEquals(b.getBd(), bd, 0.0);
    }

    @Test
    public void testGetSetBdAsMagneticFluxDensity() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bd1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensity bd2 = new MagneticFluxDensity(
                bd, MagneticFluxDensityUnit.TESLA);
        b.setBd(bd2);

        // check
        final MagneticFluxDensity bd3 = b.getBdAsMagneticFluxDensity();
        final MagneticFluxDensity bd4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd4);

        assertEquals(bd2, bd3);
        assertEquals(bd2,bd4);
    }

    @Test
    public void testSetCoordinates1() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
    }

    @Test
    public void testSetCoordinates2() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(new MagneticFluxDensity(bn, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(be, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(bd, MagneticFluxDensityUnit.TESLA));

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
    }

    @Test
    public void testSetCoordinates3() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(b.getBn(), 0.0, 0.0);
        assertEquals(b.getBe(), 0.0, 0.0);
        assertEquals(b.getBd(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA, bn, be, bd);
        b.setCoordinates(triad1);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        final MagneticFluxDensityTriad triad2 = b.getCoordinatesAsTriad();
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad3);
        assertEquals(triad1, triad2);
        assertEquals(triad1, triad3);
    }

    @Test
    public void testGetNorm() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(b.getNorm(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check
        assertEquals(b.getNorm(), Math.sqrt(bn * bn + be * be + bd * bd),
                0.0);
    }

    @Test
    public void testGetNormAsMagneticFluxDensity() {
        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity();

        // check default value
        final MagneticFluxDensity norm1 = b.getNormAsMagneticFluxDensity();
        assertEquals(norm1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(norm1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check
        final double norm = Math.sqrt(bn * bn + be * be + bd * bd);
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
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity();

        b1.copyTo(b2);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity();

        b2.copyFrom(b1);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity(
                bn, be, bd);

        final double[] array1 = b.asArray();
        final double[] array2 = new double[
                NEDMagneticFluxDensity.COMPONENTS];
        b.asArray(array2);

        // check
        assertEquals(array1[0], bn, 0.0);
        assertEquals(array1[1], be, 0.0);
        assertEquals(array1[2], bd, 0.0);
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
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b = new NEDMagneticFluxDensity(
                bn, be, bd);

        final Matrix m1 = b.asMatrix();
        final Matrix m2 = new Matrix(1, 1);
        b.asMatrix(m2);
        final Matrix m3 = new Matrix(NEDMagneticFluxDensity.COMPONENTS,
                1);
        b.asMatrix(m3);

        // check
        assertEquals(m1.getElementAtIndex(0), bn, 0.0);
        assertEquals(m1.getElementAtIndex(1), be, 0.0);
        assertEquals(m1.getElementAtIndex(2), bd, 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

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
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b2 = new NEDMagneticFluxDensity(
                bn, be, bd);
        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();

        assertTrue(b1.equals(b1, THRESHOLD));
        assertTrue(b1.equals(b2, THRESHOLD));
        assertFalse(b1.equals(b3, THRESHOLD));
        assertFalse(b1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity(
                bn, be, bd);

        final Object b2 = b1.clone();

        // check
        assertEquals(b1, b2);
    }
}
