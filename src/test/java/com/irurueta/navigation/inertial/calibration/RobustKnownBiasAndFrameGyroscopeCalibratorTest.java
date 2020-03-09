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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownBiasAndFrameGyroscopeCalibratorTest implements
        RobustKnownBiasAndFrameGyroscopeCalibratorListener {

    private static final double DEG_TO_RAD = 0.01745329252;

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustKnownBiasAndFrameGyroscopeCalibrator calibrator =
                RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);


        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and method
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates and method
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY,
                biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with measurements, bias, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX,
                biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias coordinates, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // test constructor with bias coordinates as angular speed
        final AngularSpeed bx = new AngularSpeed(biasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by = new AngularSpeed(biasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bz = new AngularSpeed(biasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        // test create with bias coordinates as angular speed, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // test create with measurements, bias coordinates as angular speed and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        // test create with measurements, bias coordinates as angular speed, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates as angular speed, common axis used and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates angular speed, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates as angular speed, common axis
        // used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias coordinates as angular speed, common axis
        // used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias array and method
        final double[] bias = bg.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with measurements, bias coordinates array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias coordinates array, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias coordinates array, common axis used and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        // test create with bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        // test create with measurements, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, bg, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        // test create with bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, bias matrix, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // test create with quality scores, measurements and method

        // RANSAC
        final double[] qualityScores = new double[
                RobustKnownBiasAndFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measures, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, measurements, bias coordinates, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        // test create with quality scores, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias coordinates, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                biasX, biasY, biasZ, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias coordinates, common axis
        // used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, measurements, biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias coordinates as angular speed and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        // test create with quality scores, bias coordinates as angular speed, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bx, by, bz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates as angular
        // speed and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        // test create with quality scores, measurements, bias coordinates as angular speed,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias coordinates as angular speed,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias coordinates as angular speed,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                qualityScores, bx, by, bz, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias coordinates as angular
        // speed, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias coordinates as angular
        // speed, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with quality scores, bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // test create with quality scores, measurements, bias array,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bias, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias array, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias array, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bias, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        // test create with quality scores, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        // test create with quality scores, measurements, bias matrix, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, bias matrix, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, bias matrix, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, bias matrix, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                measurements, bg, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateDefaultMethod() {
        RobustKnownBiasAndFrameGyroscopeCalibrator calibrator =
                RobustKnownBiasAndFrameGyroscopeCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);


        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                biasX, biasY, biasZ);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                biasX, biasY, biasZ, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                biasX, biasY, biasZ, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                biasX, biasY, biasZ, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                biasX, biasY, biasZ, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(
                measurements, biasX, biasY, biasZ, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        final AngularSpeed bx = new AngularSpeed(biasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by = new AngularSpeed(biasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bz = new AngularSpeed(biasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bx, by, bz, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAngularSpeedX(), bx);
        assertEquals(calibrator.getBiasAngularSpeedY(), by);
        assertEquals(calibrator.getBiasAngularSpeedZ(), bz);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        final double[] bias = bg.getBuffer();
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bias, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());


        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements,
                bg, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), bg);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator,
            final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator,
            final float progress) {
    }

    private Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }
}
