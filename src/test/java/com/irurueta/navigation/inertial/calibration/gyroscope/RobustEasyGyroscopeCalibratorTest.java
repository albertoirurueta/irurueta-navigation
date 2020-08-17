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

public class RobustEasyGyroscopeCalibratorTest implements
        RobustEasyGyroscopeCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
    }

    @Test
    public void testCreate2() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, initialBias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        RobustEasyGyroscopeCalibrator calibrator = RobustEasyGyroscopeCalibrator
                .create(qualityScores, sequences, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustEasyGyroscopeCalibrator.create(
                qualityScores, sequences,
                false, false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustEasyGyroscopeCalibrator);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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
        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create();

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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences, initialBias,
                        initialMg, initialGg, accelerometerBias,
                        accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getInitialBias(), initialBias, 0.0);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
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

        final RobustEasyGyroscopeCalibrator calibrator =
                RobustEasyGyroscopeCalibrator.create(sequences,
                        false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getSequences(), sequences);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getInitialBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(RobustEasyGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(RobustEasyGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(RobustEasyGyroscopeCalibrator calibrator, int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(RobustEasyGyroscopeCalibrator calibrator, float progress) {
    }
}
