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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Generates body magnetic flux density instances from true body magnetic
 * flux density values taking into account provided magnetometer errors
 * for a calibrated magnetometer.
 */
public class BodyMagneticFluxDensityGenerator {

    /**
     * Prevents instantiation of utility class.
     */
    private BodyMagneticFluxDensityGenerator() {
    }

    /**
     * Generates uncalibrated body magnetic flux densities instances for provided
     * ground-truth body magnetic flux densities.
     *
     * @param trueMagneticFluxDensities collection of ground-truth body magnetic
     *                                  flux densities.
     * @param magnetometerHardIron      magnetometer hard-iron biases. Must
     *                                  have length 3.
     * @param magnetometerSoftIron      magnetometer soft-iron and
     *                                  cross-couplings. Must be 3x3.
     * @return collection of generated uncalibrated magnetic flux densities
     * for each provided ground-truth one.
     * @throws IllegalArgumentException if either hard-iron or soft-iron doesn't
     *                                  have proper size.
     */
    public static Collection<BodyMagneticFluxDensity> generate(
            final Collection<BodyMagneticFluxDensity> trueMagneticFluxDensities,
            final double[] magnetometerHardIron, final Matrix magnetometerSoftIron) {
        final List<BodyMagneticFluxDensity> result = new ArrayList<>();
        generate(trueMagneticFluxDensities, magnetometerHardIron,
                magnetometerSoftIron, result);
        return result;
    }

    /**
     * Generates uncalibrated body magnetic flux densities instances for provided
     * ground-truth body magnetic flux densities.
     *
     * @param trueMagneticFluxDensities collection of ground-truth body magnetic
     *                                  flux densities.
     * @param magnetometerHardIron      magnetometer hard-iron biases. Must
     *                                  have length 3.
     * @param magnetometerSoftIron      magnetometer soft-iron and
     *                                  cross-couplings. Must be 3x3.
     * @param result                    collection where generated uncalibrated magnetic flux densities
     *                                  for each provided ground-truth one will be stored.
     * @throws IllegalArgumentException if either hard-iron or soft-iron doesn't
     *                                  have proper size.
     */
    public static void generate(
            final Collection<BodyMagneticFluxDensity> trueMagneticFluxDensities,
            final double[] magnetometerHardIron, final Matrix magnetometerSoftIron,
            final Collection<BodyMagneticFluxDensity> result) {
        try {
            final Matrix mBtrue = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            final Matrix identity = Matrix.identity(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            final Matrix tmp33 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            final Matrix tmp31 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);

            for (final BodyMagneticFluxDensity b : trueMagneticFluxDensities) {
                final BodyMagneticFluxDensity r = new BodyMagneticFluxDensity();

                internalGenerate(b, magnetometerHardIron,
                        magnetometerSoftIron, r, mBtrue, identity, tmp33, tmp31);

                result.add(r);
            }
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Generates an uncalibrated body magnetic flux density instance for provided
     * ground-truth body magnetic flux density.
     *
     * @param trueMagneticFluxDensity a ground-truth body magnetic flux density.
     * @param magnetometerHardIron    magnetometer hard-iron biases. Must
     *                                have length 3.
     * @param magnetometerSoftIron    magnetometer soft-iron and
     *                                cross-couplings. Must be 3x3.
     * @return an uncalibrated magnetic flux density.
     * @throws IllegalArgumentException if either hard-iron or soft-iron doesn't
     *                                  have proper size.
     */
    public static BodyMagneticFluxDensity generate(
            final BodyMagneticFluxDensity trueMagneticFluxDensity,
            final double[] magnetometerHardIron,
            final Matrix magnetometerSoftIron) {
        final BodyMagneticFluxDensity result = new BodyMagneticFluxDensity();
        generate(trueMagneticFluxDensity, magnetometerHardIron,
                magnetometerSoftIron, result);
        return result;
    }

    /**
     * Generates an uncalibrated body magnetic flux density instance for provided
     * ground-truth body magnetic flux density.
     *
     * @param trueMagneticFluxDensity a ground-truth body magnetic flux density.
     * @param magnetometerHardIron    magnetometer hard-iron biases. Must
     *                                have length 3.
     * @param magnetometerSoftIron    magnetometer soft-iron and
     *                                cross-couplings. Must be 3x3.
     * @param result                  instance where uncalibrated magnetic flux
     *                                density will be stored.
     * @throws IllegalArgumentException if either hard-iron or soft-iron doesn't
     *                                  have proper size.
     */
    public static void generate(
            final BodyMagneticFluxDensity trueMagneticFluxDensity,
            final double[] magnetometerHardIron,
            final Matrix magnetometerSoftIron,
            final BodyMagneticFluxDensity result) {
        try {
            final Matrix mBtrue = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            final Matrix identity = Matrix.identity(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            final Matrix tmp33 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            final Matrix tmp31 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            internalGenerate(trueMagneticFluxDensity, magnetometerHardIron,
                    magnetometerSoftIron, result, mBtrue, identity, tmp33, tmp31);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Generates an uncalibrated body magnetic flux density instance for provided
     * ground-truth body magnetic flux density.
     *
     * @param trueMagneticFluxDensity a ground-truth body magnetic flux density.
     * @param magnetometerHardIron    magnetometer hard-iron biases. Must
     *                                have length 3.
     * @param magnetometerSoftIron    magnetometer soft-iron and
     *                                cross-couplings. Must be 3x3.
     * @param result                  instance where uncalibrated magnetic flux
     *                                density will be stored.
     * @param mBtrue                  a 3x1 matrix to be reused to store
     *                                ground-truth body magnetic flux
     *                                density.
     * @param identity                a 3x3 identity matrix to be reused.
     * @param tmp33                   a 3x3 temporary matrix to be reused.
     * @param tmp31                   a 3x1 temporary matrix to be reused.
     * @throws WrongSizeException if any of provided matrices has invalid size.
     */
    private static void internalGenerate(
            final BodyMagneticFluxDensity trueMagneticFluxDensity,
            final double[] magnetometerHardIron,
            final Matrix magnetometerSoftIron,
            final BodyMagneticFluxDensity result,
            final Matrix mBtrue,
            final Matrix identity,
            final Matrix tmp33,
            final Matrix tmp31) throws WrongSizeException {
        if (magnetometerHardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (magnetometerSoftIron.getRows() != BodyMagneticFluxDensity.COMPONENTS ||
                magnetometerSoftIron.getColumns() != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The magnetometer model is:
        // mBmeas = bm + (I + Mm) * mBtrue + w

        trueMagneticFluxDensity.asMatrix(mBtrue);
        tmp33.copyFrom(identity);
        tmp33.add(magnetometerSoftIron);

        tmp33.multiply(mBtrue, tmp31);
        for (int i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
            tmp31.setElementAtIndex(i,
                    tmp31.getElementAtIndex(i) + magnetometerHardIron[i]);
        }

        result.setCoordinates(
                tmp31.getElementAtIndex(0),
                tmp31.getElementAtIndex(1),
                tmp31.getElementAtIndex(2));
    }
}
