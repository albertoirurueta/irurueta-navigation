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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownBiasAndFrameAccelerometerCalibratorTest implements
        RobustKnownBiasAndFrameAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustKnownBiasAndFrameAccelerometerCalibrator calibrator =
                RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);


        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and method
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates and method
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with measurements, bias, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test constructor with bias coordinates as acceleration
        final Acceleration bax = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        // test create with bias coordinates as acceleration, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates as acceleration and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        // test create with measurements, bias coordinates as acceleration, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates as acceleration, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates as acceleration, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates as acceleration, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias coordinates as acceleration, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias array and method
        final double[] bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with measurements, bias coordinates array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates array, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // test create with bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // test create with measurements, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // test create with bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements and method

        // RANSAC
        final double[] qualityScores = new double[
                RobustKnownBiasAndFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, measurements, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias coordinates, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias coordinates, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates as acceleration and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        // test create with quality scores, bias coordinates as acceleration, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates as acceleration
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        // test create with quality scores, measurements, bias coordinates as acceleration,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates as acceleration, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias coordinates as acceleration,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates as acceleration,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias coordinates as acceleration,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with quality scores, bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with quality scores, measurements, bias array,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, bias, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias array,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias array, common axis used
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bias, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // test create with quality scores, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // test create with quality scores, measurements, bias matrix, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias matrix, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, ba, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias matrix, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias matrix,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, ba, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateDefaultMethod() {
        RobustKnownBiasAndFrameAccelerometerCalibrator calibrator =
                RobustKnownBiasAndFrameAccelerometerCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);


        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                biasX, biasY, biasZ, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        final Acceleration bax = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bax, bay, baz, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bax, bay, baz, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasXAsAcceleration(), bax);
        assertEquals(calibrator.getBiasYAsAcceleration(), bay);
        assertEquals(calibrator.getBiasZAsAcceleration(), baz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        final double[] bias = ba.getBuffer();
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                bias, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, bias, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                ba, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                measurements, ba, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator,
            final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator,
            final float progress) {
    }

    private Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }
}
