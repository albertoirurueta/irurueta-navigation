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
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownBiasAndPositionAccelerometerCalibratorTest implements
        RobustKnownBiasAndPositionAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testCreateWithMethod() throws WrongSizeException {
        // create 1

        // RANSAC
        RobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);


        // create 2

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);


        // create 3
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 4

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 5
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 6

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 7
        final Matrix ma = generateMa();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 8
        final ECEFPosition ecefPosition = new ECEFPosition();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);


        // create 9

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 10

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 11

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 12

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 13

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 14

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 15

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // chekc
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 16

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 17

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 18

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 19

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 20

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 21

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 22

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 23

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 24

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 25
        final NEDPosition nedPosition = new NEDPosition();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));


        // create 26

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);


        // create 27

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 28

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 29

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 30

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 31

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 32

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // chekc
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 33

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 34

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 35

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 36

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 37

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 38

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 39

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 40

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 41

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this, RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 42
        final double[] qualityScores = new double[10];

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 43

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 44

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 45

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 46

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 47

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 48

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // chekc
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 49

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 50

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 51

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 52

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 53

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 54

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 55

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 56

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 57

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 58

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);


        // create 59

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 60

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 69

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 70

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 71

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 72

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);


        // create 73

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 74

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 75

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 76

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);


        // create 77

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 78

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 79

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 80

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 81

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        assertTrue(calibrator instanceof RANSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        assertTrue(calibrator instanceof LMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        assertTrue(calibrator instanceof MSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        assertTrue(calibrator instanceof PROSACRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements,
                true, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        assertTrue(calibrator instanceof PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreateWithDefaultMethod() throws WrongSizeException {
        // create 1
        RobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                RobustKnownBiasAndPositionAccelerometerCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

        // create 2
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);

        // create 3
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);

        // create 4
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());

        // create 5
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // create 6
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // create 7
        final Matrix ma = generateMa();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // create 8
        final ECEFPosition ecefPosition = new ECEFPosition();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // create 9
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // create 10
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // create 11
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // create 12
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // create 13
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // create 14
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // create 15
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // create 16
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // create 17
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // create 18
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // create 19
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // create 20
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // create 21
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // create 22
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // create 23
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma);

        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // create 24
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                this);

        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // create 25
        final NEDPosition nedPosition = new NEDPosition();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // create 26
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // create 27
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // create 28
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // create 29
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // create 30
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // create 31
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // create 32
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);

        // create 33
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // create 34
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // create 35
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // create 36
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);

        // create 37
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // create 38
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // create 39
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // create 40
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma);

        //check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // create 41
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {

    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {

    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator,
            final int iteration) {

    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator,
            final float progress) {

    }

    private Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private Matrix generateMa() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }
}
