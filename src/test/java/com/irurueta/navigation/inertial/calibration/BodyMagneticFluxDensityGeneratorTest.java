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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class BodyMagneticFluxDensityGeneratorTest {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_MAGNETIC_FLUX = -70e-6;
    private static final double MAX_MAGNETIC_FLUX = 70e-6;

    private static final int N = 100;

    @Test
    public void testGenerate1() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIron();

        final List<BodyMagneticFluxDensity> trueMagneticFluxDensities =
                new ArrayList<>();
        final List<BodyMagneticFluxDensity> expectedMagneticFluxDensities =
                new ArrayList<>();
        for (int i = 0; i < N; i++) {
            final BodyMagneticFluxDensity trueMagneticFluxDensity =
                    generateTruth(randomizer);
            final BodyMagneticFluxDensity expectedMagneticFluxDensity =
                    generateExpected(trueMagneticFluxDensity, hardIron,
                            softIron);

            trueMagneticFluxDensities.add(trueMagneticFluxDensity);
            expectedMagneticFluxDensities.add(expectedMagneticFluxDensity);
        }

        final Collection<BodyMagneticFluxDensity> result =
                BodyMagneticFluxDensityGenerator.generate(
                        trueMagneticFluxDensities, hardIron, softIron);

        assertEquals(expectedMagneticFluxDensities, result);
    }

    @Test
    public void testGenerate2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIron();

        final List<BodyMagneticFluxDensity> trueMagneticFluxDensities =
                new ArrayList<>();
        final List<BodyMagneticFluxDensity> expectedMagneticFluxDensities =
                new ArrayList<>();
        for (int i = 0; i < N; i++) {
            final BodyMagneticFluxDensity trueMagneticFluxDensity =
                    generateTruth(randomizer);
            final BodyMagneticFluxDensity expectedMagneticFluxDensity =
                    generateExpected(trueMagneticFluxDensity, hardIron,
                            softIron);

            trueMagneticFluxDensities.add(trueMagneticFluxDensity);
            expectedMagneticFluxDensities.add(expectedMagneticFluxDensity);
        }

        final List<BodyMagneticFluxDensity> result = new ArrayList<>();
        BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensities,
                hardIron, softIron, result);

        assertEquals(expectedMagneticFluxDensities, result);
    }

    @Test
    public void testGenerate3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIron();

        final BodyMagneticFluxDensity trueMagneticFluxDensity =
                generateTruth(randomizer);
        final BodyMagneticFluxDensity expectedMagneticFluxDensity =
                generateExpected(trueMagneticFluxDensity, hardIron,
                        softIron);

        final BodyMagneticFluxDensity result = BodyMagneticFluxDensityGenerator
                .generate(trueMagneticFluxDensity, hardIron, softIron);

        assertEquals(expectedMagneticFluxDensity, result);
    }

    @Test
    public void testGenerate4() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIron();

        final BodyMagneticFluxDensity trueMagneticFluxDensity =
                generateTruth(randomizer);
        final BodyMagneticFluxDensity expectedMagneticFluxDensity =
                generateExpected(trueMagneticFluxDensity, hardIron,
                        softIron);

        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensity,
                hardIron, softIron, result);

        assertEquals(expectedMagneticFluxDensity, result);
    }

    private static BodyMagneticFluxDensity generateTruth(
            final UniformRandomizer randomizer) {
        final double bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        final double by = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        final double bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        return new BodyMagneticFluxDensity(bx, by, bz);
    }

    private static BodyMagneticFluxDensity generateExpected(
            final BodyMagneticFluxDensity input, final double[] hardIron,
            final Matrix softIron) {
        try {
            final Matrix bTrue = input.asMatrix();
            final Matrix tmp = Matrix.identity(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            tmp.add(softIron);
            tmp.multiply(bTrue);

            tmp.setElementAtIndex(0,
                    tmp.getElementAtIndex(0) + hardIron[0]);
            tmp.setElementAtIndex(1,
                    tmp.getElementAtIndex(1) + hardIron[1]);
            tmp.setElementAtIndex(2,
                    tmp.getElementAtIndex(2) + hardIron[2]);

            return new BodyMagneticFluxDensity(
                    tmp.getElementAtIndex(0),
                    tmp.getElementAtIndex(1),
                    tmp.getElementAtIndex(2));
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static double[] generateHardIron(
            final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIron() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }
}
