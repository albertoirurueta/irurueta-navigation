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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownBiasEasyGyroscopeCalibratorTest implements
        RobustKnownBiasEasyGyroscopeCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
    }

    @Test
    public void testCreate2() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate3() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate4() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate5() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate6() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate7() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate8() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate9() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate10() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate11() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate12() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate13() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate14() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate15() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate16() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate17() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate18() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate19() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate20() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate21() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate22() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate23() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate24() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate25() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate26() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate27() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate28() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate29() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate30() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate31() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate32() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate33() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate34() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasEasyGyroscopeCalibrator calibrator = RobustKnownBiasEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testCreateWithDefaultMethod2() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod3() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod4() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod5() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod6() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod7() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod8() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod9() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod10() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod11() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod12() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod13() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod14() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod15() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod16() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod17() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasEasyGyroscopeCalibrator calibrator =
                RobustKnownBiasEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(RobustKnownBiasEasyGyroscopeCalibrator calibrator, int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(RobustKnownBiasEasyGyroscopeCalibrator calibrator, float progress) {
    }
}
