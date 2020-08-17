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
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownBiasTurntableGyroscopeCalibratorTest implements
        RobustKnownBiasTurntableGyroscopeCalibratorListener {

    private static final double ROTATION_RATE = Math.PI / 2.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator = RobustKnownBiasTurntableGyroscopeCalibrator
                .create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
    }

    @Test
    public void testCreate2() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate3() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate4() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate5() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate6() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate7() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate9() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate11() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate12() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate13() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate14() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate15() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate16() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate18() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate19() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate20() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate21() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate22() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate23() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate24() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate25() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate26() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate27() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate28() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate29() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate30() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate31() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate32() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate33() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(position, ROTATION_RATE, TIME_INTERVAL,
                                measurements, false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate34() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator = RobustKnownBiasTurntableGyroscopeCalibrator
                .create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate35() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate36() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate37() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate38() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate39() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate40() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate41() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate42() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate43() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate44() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate45() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate46() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate47() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate48() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate49() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate50() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate51() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate52() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate53() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate54() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate55() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate56() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate57() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate58() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements, initialBias,
                                initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate59() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate60() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate61() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreate62() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate63() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreate64() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate65() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // LMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // MSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROSAC
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);

        // PROMedS
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreate66() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator
                        .create(qualityScores, position, ROTATION_RATE,
                                TIME_INTERVAL, measurements,
                                false,
                                false,
                                initialBias, initialMg, initialGg,
                                accelerometerBias, accelerometerMa,
                                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod1() {
        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testCreateWithDefaultMethod2() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(position,
                        ROTATION_RATE, TIME_INTERVAL, measurements,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod3() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod4() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod5() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod6() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod7() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod9() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator = RobustKnownBiasTurntableGyroscopeCalibrator.create(
                position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false,
                false,
                initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMedS);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod11() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod12() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod13() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod14() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod15() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod16() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), position);
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod18() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod19() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod20() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod21() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod22() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod23() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), initialBias, 0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod24() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod25() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), initialBias);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                accelerometerBias);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod26() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod27() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod28() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
    }

    @Test
    public void testCreateWithDefaultMethod29() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod30() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
    }

    @Test
    public void testCreateWithDefaultMethod31() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(calibrator.getBias(), initialBias,
                0.0);
        assertEquals(calibrator.getInitialMg(), initialMg);
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(calibrator.getAccelerometerBias(),
                accelerometerBias, 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod32() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void testCreateWithDefaultMethod33() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                RobustKnownBiasTurntableGyroscopeCalibrator.create(
                        position, ROTATION_RATE, TIME_INTERVAL,
                        measurements, false,
                        false,
                        initialBias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa,
                        this);

        // check
        assertEquals(calibrator.getMethod(),
                RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(calibrator.getTurntableRotationRate(), ROTATION_RATE,
                0.0);
        assertEquals(calibrator.getTimeInterval(), TIME_INTERVAL,
                0.0);
        assertSame(calibrator.getMeasurements(), measurements);
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
    public void onCalibrateStart(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator) {

    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator) {

    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator,
            final int iteration) {

    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator,
            final float progress) {

    }
}
