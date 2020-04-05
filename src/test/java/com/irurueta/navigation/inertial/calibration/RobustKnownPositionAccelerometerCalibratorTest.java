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
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RobustKnownPositionAccelerometerCalibratorTest implements
        RobustKnownPositionAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testCreateWithMethod() throws WrongSizeException {

        // create 1

        // RANSAC
        RobustKnownPositionAccelerometerCalibrator calibrator =
                RobustKnownPositionAccelerometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);


        // create 2

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);


        // create 3
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 4

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 5
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 6

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 7
        final Matrix ma = generateMa();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 8
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);


        // create 9

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 10

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 11

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 12

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 13

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 14

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 15

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 16

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 17

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 18

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 19

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 20

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 21

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 22

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 23

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 24

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 25

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));


        // create 26

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);


        // create 27

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 28

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 29

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 30

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 31

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 32

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 33

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 34

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 35

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 36

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 37

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 38

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 39

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 40

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 41

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 42
        final double[] qualityScores = new double[13];

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // create 43

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 44

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 45

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 46

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 47

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 48

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 49

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, true, bias,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 50

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 51

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 52

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 53

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 54

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 55

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 56

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 57

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                ecefPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 58

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);


        // create 59

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // create 60

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 61

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 62

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 63

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 64

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, true, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, true, bias,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, true, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(
                qualityScores, nedPosition, measurements, true, bias,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 65

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROmedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, bias, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 66

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 67

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 68

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 69

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 70

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 71

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 72

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 73

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores,
                nedPosition, measurements, true, ba, ma, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);

    }

    @Test
    public void testCreateWithDefaultMethod() throws WrongSizeException {

        // create 1
        RobustKnownPositionAccelerometerCalibrator calibrator = RobustKnownPositionAccelerometerCalibrator
                .create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);


        // create 2
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);


        // create 3
        List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);

        // create 4
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 5
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 6
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // calibrate 7
        final Matrix ma = generateMa();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // calibrate 8
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);


        // calibrate 9
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);


        // calibrate 10
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // calibrate 11
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // create 12
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // create 13
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 14
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 15
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // create 16
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // create 17
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 18
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 19
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 20
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 21
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 22
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 23
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 24
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition,
                measurements, true, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 25
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));


        // calibrate 26
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);


        // calibrate 27
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // calibrate 28
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // calibrate 29
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // calibrate 30
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // calibrate 31
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // calibrate 32
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, bias);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);


        // calibrate 33
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, bias, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(calibrator.getListener(), this);


        // calibrate 34
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // calibrate 35
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 36
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, ba);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);


        // create 37
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, ba, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertSame(calibrator.getListener(), this);


        // create 38
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 39
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);


        // create 40
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, ba, ma);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);


        // create 41
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition,
                measurements, true, ba, ma, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);
        assertEquals(calibrator.getInitialMa(), ma);
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownPositionAccelerometerCalibrator calibrator) {

    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownPositionAccelerometerCalibrator calibrator) {

    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAccelerometerCalibrator calibrator,
            final int iteration) {

    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAccelerometerCalibrator calibrator,
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
