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
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class IMUErrorsTest {

    private static final int COMPONENTS = 3;

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructors() throws WrongSizeException {
        // test constructor 1
        IMUErrors errors = new IMUErrors();

        // check default values
        final double[] zeros = new double[COMPONENTS];
        assertArrayEquals(errors.getAccelerometerBiases(), zeros, 0.0);
        assertArrayEquals(errors.getGyroBiases(), zeros, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                Matrix.identity(COMPONENTS, COMPONENTS));
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                Matrix.identity(COMPONENTS, COMPONENTS));
        assertEquals(errors.getGyroGDependentBiases(),
                new Matrix(COMPONENTS, COMPONENTS));
        assertEquals(errors.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(), 0.0, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(), 0.0, 0.0);

        // test constructor 2
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        errors = new IMUErrors(accelerometerBiases,
                gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(),
                new Matrix(COMPONENTS, COMPONENTS));
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(), 0.0, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(), 0.0, 0.0);

        // Force IllegalArgumentException
        double[] wrongArray = new double[1];
        final Matrix wrongMatrix1 = new Matrix(1, 3);
        final Matrix wrongMatrix2 = new Matrix(4, 1);

        errors = null;
        try {
            errors = new IMUErrors(wrongArray,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    wrongArray, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, wrongMatrix1,
                    gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, wrongMatrix2,
                    gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix1, accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix2, accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 3
        final Matrix accelerometerBiasesMatrix = new Matrix(3, 1);
        accelerometerBiasesMatrix.fromArray(accelerometerBiases);

        final Matrix gyroBiasesMatrix = new Matrix(3, 1);
        gyroBiasesMatrix.fromArray(gyroBiases);

        errors = new IMUErrors(accelerometerBiasesMatrix,
                gyroBiasesMatrix, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(),
                new Matrix(COMPONENTS, COMPONENTS));
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(), 0.0, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(), 0.0, 0.0);

        // Force IllegalArgumentException
        errors = null;
        try {
            errors = new IMUErrors(wrongMatrix1, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(wrongMatrix2, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, wrongMatrix1,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, wrongMatrix2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 4
        final Acceleration[] accelerometerBiases2 = new Acceleration[COMPONENTS];
        accelerometerBiases2[0] = new Acceleration(accelerometerBiases[0],
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        accelerometerBiases2[1] = new Acceleration(accelerometerBiases[1],
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        accelerometerBiases2[2] = new Acceleration(accelerometerBiases[2],
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final AngularSpeed[] gyroBiases2 = new AngularSpeed[COMPONENTS];
        gyroBiases2[0] = new AngularSpeed(gyroBiases[0],
                AngularSpeedUnit.RADIANS_PER_SECOND);
        gyroBiases2[1] = new AngularSpeed(gyroBiases[1],
                AngularSpeedUnit.RADIANS_PER_SECOND);
        gyroBiases2[2] = new AngularSpeed(gyroBiases[2],
                AngularSpeedUnit.RADIANS_PER_SECOND);

        errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(),
                new Matrix(COMPONENTS, COMPONENTS));
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(), 0.0, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(), 0.0, 0.0);

        // Force IllegalArgumentException
        final Acceleration[] wrongAccelerations = new Acceleration[1];
        final AngularSpeed[] wrongAngularSpeed = new AngularSpeed[1];

        errors = null;
        try {
            errors = new IMUErrors(wrongAccelerations, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, wrongAngularSpeed,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix1, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix2, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 5
        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);

        // Force IllegalArgumentException
        errors = null;
        try {
            errors = new IMUErrors(wrongArray,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    wrongArray, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, wrongMatrix1,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, wrongMatrix2,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix1, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix2, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases,
                    gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 6
        errors = new IMUErrors(accelerometerBiasesMatrix,
                gyroBiasesMatrix, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                accelerometerQuantizationLevel, gyroQuantizationLevel);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);

        // Force IllegalArgumentException
        errors = null;
        try {
            errors = new IMUErrors(wrongMatrix1, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(wrongMatrix2, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, wrongMatrix1,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, wrongMatrix2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel, gyroQuantizationLevel);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 7
        final Acceleration accelerometerQuantizationLevel2 =
                new Acceleration(accelerometerQuantizationLevel,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed gyroQuantizationLevel2 =
                new AngularSpeed(gyroQuantizationLevel,
                        AngularSpeedUnit.RADIANS_PER_SECOND);
        errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel2,
                gyroQuantizationLevel2);

        // check default values
        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);

        // Force IllegalArgumentException
        errors = null;
        try {
            errors = new IMUErrors(wrongAccelerations, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, wrongAngularSpeed,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors,
                    gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix1, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    wrongMatrix2, gyroGDependenciesBiases,
                    accelerometerNoiseRootPSD, gyroNoiseRootPSD,
                    accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    wrongMatrix1, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD, accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors = new IMUErrors(accelerometerBiases2, gyroBiases2,
                    accelerometerScaleFactorAndCrossCouplingErrors,
                    gyroScaleFactorAndCrossCouplingErrors,
                    wrongMatrix2, accelerometerNoiseRootPSD,
                    gyroNoiseRootPSD, accelerometerQuantizationLevel2,
                    gyroQuantizationLevel2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(errors);


        // test constructor 8
        errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final IMUErrors errors2 = new IMUErrors(errors);

        // check default values
        assertArrayEquals(errors2.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors2.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors2.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors2.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors2.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors2.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors2.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiases() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final double[] zeros = new double[COMPONENTS];
        assertArrayEquals(errors.getAccelerometerBiases(), zeros, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerBiases(accelerometerBiases);


        assertArrayEquals(errors.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        final double[] result = new double[COMPONENTS];
        errors.getAccelerometerBiases(result);
        assertArrayEquals(result, accelerometerBiases, 0.0);


        // Force IllegalArgumentException
        try {
            errors.setAccelerometerBiases(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.getAccelerometerBiases(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerBiasesAsMatrix() throws WrongSizeException {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getAccelerometerBiasesAsMatrix(),
                new Matrix(COMPONENTS, 1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] array = new double[COMPONENTS];
        randomizer.fill(array, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerBiases1 = Matrix.newFromArray(array);

        errors.setAccelerometerBiases(accelerometerBiases1);

        // check
        final Matrix accelerometerBiases2 = new Matrix(COMPONENTS, 1);
        errors.getAccelerometerBiasesAsMatrix(accelerometerBiases2);
        final Matrix accelerometerBiases3 = errors.getAccelerometerBiasesAsMatrix();

        assertEquals(accelerometerBiases1, accelerometerBiases2);
        assertEquals(accelerometerBiases1, accelerometerBiases3);


        // Force IllegalArgumentException
        try {
            errors.getAccelerometerBiasesAsMatrix(new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setAccelerometerBiases(new Matrix(COMPONENTS, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setAccelerometerBiases(new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerBiasesAsAcceleration() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final Acceleration[] accelerations1 = errors.getAccelerometerBiasesAsAcceleration();
        for (Acceleration a : accelerations1) {
            assertEquals(a, new Acceleration(0.0,
                    AccelerationUnit.METERS_PER_SQUARED_SECOND));
        }

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Acceleration[] accelerations2 = new Acceleration[COMPONENTS];
        for (int i = 0; i < COMPONENTS; i++) {
            accelerations2[i] = new Acceleration(
                    randomizer.nextDouble(MIN_VALUE, MAX_VALUE),
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
        }

        errors.setAccelerometerBiases(accelerations2);

        // check
        final Acceleration[] accelerations3 = errors.getAccelerometerBiasesAsAcceleration();
        final Acceleration[] accelerations4 = new Acceleration[COMPONENTS];
        accelerations4[0] = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        errors.getAccelerometerBiasesAsAcceleration(accelerations4);

        assertArrayEquals(accelerations2, accelerations3);
        assertArrayEquals(accelerations2, accelerations4);

        // Force IllegalArgumentException
        try {
            errors.getAccelerometerBiasesAsAcceleration(new Acceleration[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setAccelerometerBiases(new Acceleration[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroBiases() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final double[] zeros = new double[COMPONENTS];
        assertArrayEquals(errors.getGyroBiases(), zeros, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        errors.setGyroBiases(gyroBiases);


        assertArrayEquals(errors.getGyroBiases(), gyroBiases, 0.0);
        final double[] result = new double[COMPONENTS];
        errors.getGyroBiases(result);
        assertArrayEquals(result, gyroBiases, 0.0);


        // Force IllegalArgumentException
        try {
            errors.setGyroBiases(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.getGyroBiases(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroBiasesAsMatrix() throws WrongSizeException {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroBiasesAsMatrix(),
                new Matrix(COMPONENTS, 1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] array = new double[COMPONENTS];
        randomizer.fill(array, MIN_VALUE, MAX_VALUE);

        final Matrix gyroBiases1 = Matrix.newFromArray(array);

        errors.setGyroBiases(gyroBiases1);

        // check
        final Matrix gyroBiases2 = new Matrix(COMPONENTS, 1);
        errors.getGyroBiasesAsMatrix(gyroBiases2);
        final Matrix gyroBiases3 = errors.getGyroBiasesAsMatrix();

        assertEquals(gyroBiases1, gyroBiases2);
        assertEquals(gyroBiases1, gyroBiases3);

        // Force IllegalArgumentException
        try {
            errors.getGyroBiasesAsMatrix(new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setGyroBiases(new Matrix(COMPONENTS, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setGyroBiases(new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroBiasesAsAngularSpeed() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final AngularSpeed[] array1 = errors.getGyroBiasesAsAngularSpeed();
        for (AngularSpeed as : array1) {
            assertEquals(as, new AngularSpeed(0.0,
                    AngularSpeedUnit.RADIANS_PER_SECOND));
        }

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final AngularSpeed[] array2 = new AngularSpeed[COMPONENTS];
        for (int i = 0; i < COMPONENTS; i++) {
            array2[i] = new AngularSpeed(
                    randomizer.nextDouble(MIN_VALUE, MAX_VALUE),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
        }

        errors.setGyroBiases(array2);

        // check
        final AngularSpeed[] array3 = errors.getGyroBiasesAsAngularSpeed();
        final AngularSpeed[] array4 = new AngularSpeed[COMPONENTS];
        array4[0] = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        errors.getGyroBiasesAsAngularspeed(array4);

        assertArrayEquals(array2, array3);
        assertArrayEquals(array2, array4);

        // Force IllegalArgumentException
        try {
            errors.getGyroBiasesAsAngularspeed(new AngularSpeed[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setGyroBiases(new AngularSpeed[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerScaleFactorAndCrossCouplingErrors()
            throws WrongSizeException {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                Matrix.identity(COMPONENTS, COMPONENTS));

        // set new value
        final Matrix ma1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, ma1);

        errors.setAccelerometerScaleFactorAndCrossCouplingErrors(ma1);

        // check
        final Matrix ma2 = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();
        final Matrix ma3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getAccelerometerScaleFactorAndCrossCouplingErrors(ma3);

        assertEquals(ma1, ma2);
        assertEquals(ma1, ma3);

        // Force IllegalArgumentException
        try {
            errors.setAccelerometerScaleFactorAndCrossCouplingErrors(
                    new Matrix(COMPONENTS, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setAccelerometerScaleFactorAndCrossCouplingErrors(
                    new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroScaleFactorAndCrossCouplingErrors()
            throws WrongSizeException {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroScaleFactorAndCrossCouplingErrors(),
                Matrix.identity(COMPONENTS, COMPONENTS));

        // set new value
        final Matrix mg1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, mg1);

        errors.setGyroScaleFactorAndCrossCouplingErrors(mg1);

        // check
        final Matrix mg2 = errors.getGyroScaleFactorAndCrossCouplingErrors();
        final Matrix mg3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getGyroScaleFactorAndCrossCouplingErrors(mg3);

        assertEquals(mg1, mg2);
        assertEquals(mg1, mg3);

        // Force IllegalArgumentException
        try {
            errors.setGyroScaleFactorAndCrossCouplingErrors(
                    new Matrix(COMPONENTS, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setGyroScaleFactorAndCrossCouplingErrors(
                    new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroGDependentBiases() throws WrongSizeException {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroGDependentBiases(),
                new Matrix(COMPONENTS, COMPONENTS));

        // set new value
        final Matrix mf1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, mf1);

        errors.setGyroGDependentBiases(mf1);

        // check
        final Matrix mf2 = errors.getGyroGDependentBiases();
        final Matrix mf3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getGyroGDependentBiases(mf3);

        assertEquals(mf1, mf2);
        assertEquals(mf1, mf3);

        // Force IllegalArgumentException
        try {
            errors.setGyroGDependentBiases(new Matrix(COMPONENTS, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            errors.setGyroGDependentBiases(new Matrix(1, COMPONENTS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerNoiseRootPSD() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getAccelerometerNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerNoiseRootPSD = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);

        // check
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors.getAccelerometerNoisePSD(),
                accelerometerNoiseRootPSD * accelerometerNoiseRootPSD,
                0.0);
    }

    @Test
    public void testGetSetAccelerometerNoisePSD() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getAccelerometerNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerNoisePSD = randomizer
                .nextDouble(0.0, MAX_VALUE);

        errors.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(errors.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                ABSOLUTE_ERROR);
        assertEquals(errors.getAccelerometerNoiseRootPSD(),
                Math.sqrt(accelerometerNoisePSD), 0.0);

        // Force IllegalArgumentException
        try {
            errors.setAccelerometerNoisePSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroNoiseRootPSD() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getGyroNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setGyroNoiseRootPSD(gyroNoiseRootPSD);

        // check
        assertEquals(errors.getGyroNoiseRootPSD(),
                gyroNoiseRootPSD, 0.0);
        assertEquals(errors.getGyroNoisePSD(),
                gyroNoiseRootPSD * gyroNoiseRootPSD, 0.0);
    }

    @Test
    public void testGetSetGyroNoisePSD() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(errors.getGyroNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(0.0, MAX_VALUE);

        errors.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(errors.getGyroNoisePSD(), gyroNoisePSD, ABSOLUTE_ERROR);
        assertEquals(errors.getGyroNoiseRootPSD(),
                Math.sqrt(gyroNoisePSD), 0.0);

        // Force IllegalArgumentException
        try {
            errors.setGyroNoisePSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerQuantizationLevel() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getAccelerometerQuantizationLevel(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double level = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerQuantizationLevel(level);

        // check
        assertEquals(errors.getAccelerometerQuantizationLevel(), level, 0.0);
    }

    @Test
    public void testGetSetAccelerometerQuantizationLevelAsAcceleration() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final Acceleration level1 = errors
                .getAccelerometerQuantizationLevelAsAcceleration();

        assertEquals(level1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(level1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration level2 = new Acceleration(value,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        errors.setAccelerometerQuantizationLevel(level2);

        // check
        final Acceleration level3 = errors
                .getAccelerometerQuantizationLevelAsAcceleration();
        final Acceleration level4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        errors.getAccelerometerQuantizationLevelAsAcceleration(level4);

        assertEquals(level2, level3);
        assertEquals(level2, level4);
    }

    @Test
    public void testGetSetGyroQuantizationLevel() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        assertEquals(errors.getGyroQuantizationLevel(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double level = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setGyroQuantizationLevel(level);

        // check
        assertEquals(errors.getGyroQuantizationLevel(), level, 0.0);
    }

    @Test
    public void testGyroQuantizationLevelAsAngularSpeed() {
        final IMUErrors errors = new IMUErrors();

        // check default value
        final AngularSpeed level1 = errors.getGyroQuantizationLevelAsAngularSpeed();

        assertEquals(level1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(level1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final AngularSpeed level2 = new AngularSpeed(value,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        errors.setGyroQuantizationLevel(level2);

        // check
        final AngularSpeed level3 = errors.getGyroQuantizationLevelAsAngularSpeed();
        final AngularSpeed level4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        errors.getGyroQuantizationLevelAsAngularSpeed(level4);

        assertEquals(level2, level3);
        assertEquals(level2, level4);
    }

    @Test
    public void testCopyTo() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        final IMUErrors errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final IMUErrors errors2 = new IMUErrors();

        // copy
        errors.copyTo(errors2);

        // check
        assertArrayEquals(errors2.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors2.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors2.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors2.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors2.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors2.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors2.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);
    }

    @Test
    public void testCopyFrom() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        final IMUErrors errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final IMUErrors errors2 = new IMUErrors();

        // copy
        errors2.copyFrom(errors);

        // check
        assertArrayEquals(errors2.getAccelerometerBiases(), accelerometerBiases,
                0.0);
        assertArrayEquals(errors2.getGyroBiases(), gyroBiases, 0.0);
        assertEquals(errors2.getAccelerometerScaleFactorAndCrossCouplingErrors(),
                accelerometerScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroScaleFactorAndCrossCouplingErrors(),
                gyroScaleFactorAndCrossCouplingErrors);
        assertEquals(errors2.getGyroGDependentBiases(), gyroGDependenciesBiases);
        assertEquals(errors2.getAccelerometerNoiseRootPSD(),
                accelerometerNoiseRootPSD, 0.0);
        assertEquals(errors2.getGyroNoiseRootPSD(), gyroNoiseRootPSD, 0.0);
        assertEquals(errors2.getAccelerometerQuantizationLevel(),
                accelerometerQuantizationLevel, 0.0);
        assertEquals(errors2.getGyroQuantizationLevel(),
                gyroQuantizationLevel, 0.0);
    }

    @Test
    public void testHashCode() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        final IMUErrors errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final IMUErrors errors2 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final IMUErrors errors3 = new IMUErrors();

        assertEquals(errors1.hashCode(), errors2.hashCode());
        assertNotEquals(errors1.hashCode(), errors3.hashCode());
    }

    @Test
    public void testEquals() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        final IMUErrors errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final IMUErrors errors2 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final IMUErrors errors3 = new IMUErrors();

        //noinspection SimplifiableJUnitAssertion,EqualsWithItself
        assertTrue(errors1.equals(errors1));
        //noinspection SimplifiableJUnitAssertion
        assertTrue(errors1.equals(errors2));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(errors1.equals(errors3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(errors1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(errors1.equals(new Object()));
    }

    @Test
    public void testClone() throws CloneNotSupportedException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final double[] gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final Matrix accelerometerScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                accelerometerScaleFactorAndCrossCouplingErrors);

        final Matrix gyroScaleFactorAndCrossCouplingErrors =
                new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroScaleFactorAndCrossCouplingErrors);

        final Matrix gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE,
                gyroGDependenciesBiases);

        final double accelerometerNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double gyroNoiseRootPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final double accelerometerQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroQuantizationLevel = randomizer
                .nextDouble(MIN_VALUE, MAX_VALUE);

        final IMUErrors errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final Object errors2 = errors1.clone();

        assertEquals(errors1, errors2);
    }
}
